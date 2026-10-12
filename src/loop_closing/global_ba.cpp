// =============================================================================
// GlobalBundleAdjuster implementation — full BA in a background thread
// =============================================================================

#include "slamforge/loop_closing/global_ba.h"

#ifdef SLAMFORGE_HAS_CERES

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "optimization/ceres_compat.h"
#include "slamforge/core/camera.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"
#include "slamforge/geometry/se3.h"

namespace slamforge::loop_closing {

// ── Pinhole reprojection of undistorted observations ─────────────────────────

struct GlobalBAProjectionFunctor {
    GlobalBAProjectionFunctor(double ox, double oy, double fx, double fy, double cx, double cy)
        : ox_(ox), oy_(oy), fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

    template <typename T>
    bool operator()(const T* const q, const T* const t, const T* const pt, T* res) const {
        T rotated[3];
        ceres::UnitQuaternionRotatePoint(q, pt, rotated);
        T pc[3] = {rotated[0] + t[0], rotated[1] + t[1], rotated[2] + t[2]};
        // Reject LM trial steps across the camera plane rather than dividing
        // by zero or accepting a reprojection of a point behind the camera.
        if (pc[2] <= T(1e-6)) {
            return false;
        }
        T inv_z = T(1.0) / pc[2];
        T proj_x = T(fx_) * pc[0] * inv_z + T(cx_);
        T proj_y = T(fy_) * pc[1] * inv_z + T(cy_);
        res[0] = proj_x - T(ox_);
        res[1] = proj_y - T(oy_);
        return true;
    }

    double ox_, oy_, fx_, fy_, cx_, cy_;
};

GlobalBundleAdjuster::~GlobalBundleAdjuster() {
    Stop();
}

void GlobalBundleAdjuster::Start(Map& map, const Camera& camera, const GlobalBAConfig& config) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

    // A previous solve can have finished while its std::thread is still
    // joinable.  Always stop/join it before replacing thread_.  If it is
    // currently solving, this waits for its iteration callback to honour the
    // stop request instead of silently dropping the new BA request.
    stop_requested_ = true;
    if (thread_.joinable()) {
        thread_.join();
    }

    map_ = &map;
    camera_ = &camera;
    config_ = config;
    stop_requested_ = false;
    is_finished_ = false;
    iterations_ = 0;
    final_cost_ = 0.0;

    running_ = true;
    try {
        thread_ = std::thread([this] {
            try {
                Run();
            } catch (const std::exception& e) {
                std::cerr << "[GlobalBA] Worker exception: " << e.what() << '\n';
            } catch (...) {
                std::cerr << "[GlobalBA] Worker exception: unknown error\n";
            }
            // A background exception must not terminate the process.  These flags
            // also make the worker safely restartable after any completion path.
            running_ = false;
            is_finished_ = true;
        });
    } catch (...) {
        running_ = false;
        is_finished_ = true;
        throw;
    }
}

void GlobalBundleAdjuster::Stop() {
    RequestStop();
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (thread_.joinable()) {
        thread_.join();
    }
}

void GlobalBundleAdjuster::RequestStop() {
    stop_requested_ = true;
}

void GlobalBundleAdjuster::Run() {
    if (!map_ || !camera_ || stop_requested_ || config_.max_iterations <= 0 ||
        (config_.use_huber &&
         (!std::isfinite(config_.huber_delta) || config_.huber_delta <= 0.0))) {
        return;
    }

    // Ceres works on a snapshot and writes its optimized poses/landmarks back
    // at the end.  Holding the graph transaction for this solve prevents a
    // concurrent tracker/local-BA/loop correction from changing associations
    // underneath that snapshot or overwriting its results midway through the
    // final commit.
    auto graph_lock = map_->AcquireGraphLock();
    if (stop_requested_) {
        return;
    }

    // ── Build the Ceres problem ─────────────────────────────────────────────

    std::unique_ptr<ceres::LossFunction> loss;
    if (config_.use_huber) {
        loss = std::make_unique<ceres::HuberLoss>(config_.huber_delta);
    }
    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(problem_options);

    double fx = camera_->fx();
    double fy = camera_->fy();
    double cx = camera_->cx();
    double cy = camera_->cy();

    struct KfData {
        double q[4] = {1, 0, 0, 0};
        double t[3] = {0, 0, 0};
    };

    struct MpData {
        double point[3] = {0, 0, 0};
    };

    std::unordered_map<KeyFrame*, std::unique_ptr<KfData>> kf_data_map;
    std::unordered_map<MapPoint*, std::unique_ptr<MpData>> mp_data_map;

    // Add all keyframes to the problem
    auto all_kfs = map_->GetAllKeyFrames();
    std::erase_if(all_kfs, [](const auto& kf) {
        return !kf || kf->IsBad() || !kf->Pose().matrix().allFinite();
    });
    std::sort(all_kfs.begin(), all_kfs.end(),
              [](const auto& lhs, const auto& rhs) { return lhs->Id().id < rhs->Id().id; });
    all_kfs.erase(std::unique(all_kfs.begin(), all_kfs.end()), all_kfs.end());

    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;

        auto data = std::make_unique<KfData>();
        const SE3& Tcw = kf->Pose();
        Eigen::Quaterniond quat(Tcw.rotation());
        data->q[0] = quat.w();
        data->q[1] = quat.x();
        data->q[2] = quat.y();
        data->q[3] = quat.z();
        data->t[0] = Tcw.translation().x();
        data->t[1] = Tcw.translation().y();
        data->t[2] = Tcw.translation().z();

        optimization::detail::AddQuaternionParameterBlock(problem, data->q);
        problem.AddParameterBlock(data->t, 3);

        kf_data_map[kf.get()] = std::move(data);
    }

    // Only finite, front-facing measurements contribute to triangulation.
    // Count distinct observing keyframes, not raw association entries: a
    // one-view landmark is an unconstrained depth variable in the Schur solve.
    struct Observation {
        KeyFrame* kf;
        MapPoint* mp;
        Vec2 pixel;
    };
    std::vector<Observation> observations;
    std::unordered_map<MapPoint*, int> observation_counts;
    for (const auto& kf : all_kfs) {
        std::unordered_set<MapPoint*> observed;
        const auto& keypoints = kf->KeyPointsUndistorted();
        for (int index = 0; index < kf->NumKeyPoints(); ++index) {
            auto mp = map_->GetMapPoint(kf->MapPointIdAt(index));
            if (!mp || mp->IsBad() || static_cast<size_t>(index) >= keypoints.size()) {
                continue;
            }
            const Vec3 point_camera = kf->Pose() * mp->Position();
            const auto& pixel = keypoints[static_cast<size_t>(index)].pt;
            if (!point_camera.allFinite() || point_camera.z() <= 1e-6 || !std::isfinite(pixel.x) ||
                !std::isfinite(pixel.y) || !observed.insert(mp.get()).second) {
                continue;
            }
            observations.push_back({kf.get(), mp.get(), Vec2(pixel.x, pixel.y)});
            ++observation_counts[mp.get()];
        }
    }

    // Add all map points
    auto all_mps = map_->GetAllMapPoints();
    for (auto& mp : all_mps) {
        if (!mp || mp->IsBad() || observation_counts[mp.get()] < 2)
            continue;

        auto data = std::make_unique<MpData>();
        Vec3 pos = mp->Position();
        data->point[0] = pos.x();
        data->point[1] = pos.y();
        data->point[2] = pos.z();

        problem.AddParameterBlock(data->point, 3);
        mp_data_map[mp.get()] = std::move(data);
    }

    // Add reprojection residuals
    int total_residuals = 0;
    std::unordered_set<KeyFrame*> active_keyframes;

    for (const auto& observation : observations) {
        auto mp_it = mp_data_map.find(observation.mp);
        if (mp_it == mp_data_map.end()) {
            continue;
        }
        const auto& pose = kf_data_map.at(observation.kf);
        auto* cost = new ceres::AutoDiffCostFunction<GlobalBAProjectionFunctor, 2, 4, 3, 3>(
            new GlobalBAProjectionFunctor(observation.pixel.x(), observation.pixel.y(), fx, fy, cx,
                                          cy));
        problem.AddResidualBlock(cost, loss.get(), pose->q, pose->t, mp_it->second->point);
        active_keyframes.insert(observation.kf);
        ++total_residuals;
    }

    if (total_residuals == 0) {
        return;
    }

    // An empty/bad first keyframe must not steal the world-frame gauge from
    // the cameras that actually participate in this solve.
    bool anchored = false;
    for (const auto& kf : all_kfs) {
        const bool active = active_keyframes.contains(kf.get());
        if (!active || !anchored) {
            const auto& data = kf_data_map.at(kf.get());
            problem.SetParameterBlockConstant(data->q);
            problem.SetParameterBlockConstant(data->t);
        }
        anchored = anchored || active;
    }

    // Callback to check stop flag after each iteration
    class StopCallback : public ceres::IterationCallback {
    public:
        explicit StopCallback(std::atomic<bool>& flag) : flag_(flag) {}
        ceres::CallbackReturnType operator()(const ceres::IterationSummary&) override {
            if (flag_)
                return ceres::SOLVER_ABORT;
            return ceres::SOLVER_CONTINUE;
        }

    private:
        std::atomic<bool>& flag_;
    };

    StopCallback stop_cb(stop_requested_);

    // Solve
    ceres::Solver::Options solver_opts;
    solver_opts.linear_solver_type = ceres::SPARSE_SCHUR;
    solver_opts.max_num_iterations = config_.max_iterations;
    solver_opts.minimizer_progress_to_stdout = false;
    solver_opts.callbacks.push_back(&stop_cb);
    solver_opts.num_threads = 4;

    ceres::Solver::Summary summary;
    ceres::Solve(solver_opts, &problem, &summary);

    iterations_ = static_cast<int>(summary.iterations.size());
    final_cost_ = summary.final_cost;

    // Validate the complete snapshot before committing any of it. A cancelled
    // or unusable solve must not publish a partial trajectory into the map.
    if (stop_requested_ || !summary.IsSolutionUsable() || !std::isfinite(summary.final_cost) ||
        (std::isfinite(summary.initial_cost) && summary.final_cost > summary.initial_cost * 1.01)) {
        return;
    }
    for (const auto& [_, data] : kf_data_map) {
        const Eigen::Map<const Vec4> quaternion(data->q);
        const Eigen::Map<const Vec3> translation(data->t);
        if (!quaternion.allFinite() || quaternion.norm() < 1e-8 || !translation.allFinite()) {
            return;
        }
    }
    for (const auto& [_, data] : mp_data_map) {
        if (!Eigen::Map<const Vec3>(data->point).allFinite()) {
            return;
        }
    }

    // Update keyframe poses
    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;

        auto it = kf_data_map.find(kf.get());
        if (it == kf_data_map.end())
            continue;

        const auto& data = it->second;
        if (problem.IsParameterBlockConstant(data->q)) {
            continue;
        }
        Eigen::Quaterniond quat(data->q[0], data->q[1], data->q[2], data->q[3]);
        quat.normalize();

        SE3 Tcw = SE3::Identity();
        Tcw.linear() = quat.toRotationMatrix();
        Tcw.translation() = Vec3(data->t[0], data->t[1], data->t[2]);
        kf->SetPose(Tcw);
    }

    // Update map point positions
    for (auto& mp : all_mps) {
        if (!mp || mp->IsBad())
            continue;

        auto it = mp_data_map.find(mp.get());
        if (it == mp_data_map.end())
            continue;

        const auto& pt = it->second->point;
        mp->SetPosition(Vec3(pt[0], pt[1], pt[2]));
    }
}

}  // namespace slamforge::loop_closing

#else  // !SLAMFORGE_HAS_CERES

namespace slamforge::loop_closing {

GlobalBundleAdjuster::~GlobalBundleAdjuster() {
    Stop();
}

void GlobalBundleAdjuster::Start(Map&, const Camera&, const GlobalBAConfig&) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (thread_.joinable())
        thread_.join();
    stop_requested_ = false;
    running_ = false;
    is_finished_ = true;
}

void GlobalBundleAdjuster::Stop() {
    RequestStop();
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (thread_.joinable())
        thread_.join();
}

void GlobalBundleAdjuster::RequestStop() {
    stop_requested_ = true;
}

}  // namespace slamforge::loop_closing

#endif  // SLAMFORGE_HAS_CERES
