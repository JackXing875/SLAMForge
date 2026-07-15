// =============================================================================
// GlobalBundleAdjuster implementation — full BA in a background thread
// =============================================================================

#include "litevo/loop_closing/global_ba.h"

#ifdef LITEVO_HAS_CERES

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <exception>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "litevo/core/camera.h"
#include "litevo/core/keyframe.h"
#include "litevo/core/map.h"
#include "litevo/core/map_point.h"
#include "litevo/geometry/se3.h"

namespace litevo::loop_closing {

// ── Reprojection cost functor (same as local BA) ──────────────────────────────

struct GlobalBAProjectionFunctor {
    GlobalBAProjectionFunctor(double ox, double oy, double fx, double fy, double cx, double cy)
        : ox_(ox), oy_(oy), fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

    template <typename T>
    bool operator()(const T* const q, const T* const t, const T* const pt, T* res) const {
        T rotated[3];
        ceres::UnitQuaternionRotatePoint(q, pt, rotated);
        T pc[3] = {rotated[0] + t[0], rotated[1] + t[1], rotated[2] + t[2]};
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
    if (!map_ || !camera_) {
        return;
    }

    // Ceres works on a snapshot and writes its optimized poses/landmarks back
    // at the end.  Holding the graph transaction for this solve prevents a
    // concurrent tracker/local-BA/loop correction from changing associations
    // underneath that snapshot or overwriting its results midway through the
    // final commit.
    auto graph_lock = map_->AcquireGraphLock();

    // ── Build the Ceres problem ─────────────────────────────────────────────

    ceres::Problem problem;

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
    bool first_kf = true;

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

        problem.AddParameterBlock(data->q, 4, new ceres::QuaternionManifold());
        problem.AddParameterBlock(data->t, 3);

        if (first_kf) {
            problem.SetParameterBlockConstant(data->q);
            problem.SetParameterBlockConstant(data->t);
            first_kf = false;
        }

        kf_data_map[kf.get()] = std::move(data);
    }

    // Add all map points
    auto all_mps = map_->GetAllMapPoints();
    for (auto& mp : all_mps) {
        if (!mp || mp->IsBad())
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
    ceres::LossFunction* loss = nullptr;
    if (config_.use_huber) {
        loss = new ceres::HuberLoss(config_.huber_delta);
    }

    int total_residuals = 0;

    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;

        auto kf_it = kf_data_map.find(kf.get());
        if (kf_it == kf_data_map.end())
            continue;

        for (int idx = 0; idx < kf->NumKeyPoints(); ++idx) {
            MapPointId mp_id = kf->MapPointIdAt(idx);
            if (mp_id.id == 0)
                continue;

            auto mp = map_->GetMapPoint(mp_id);
            if (!mp || mp->IsBad())
                continue;

            auto mp_it = mp_data_map.find(mp.get());
            if (mp_it == mp_data_map.end())
                continue;

            const auto& kps = kf->KeyPoints();
            if (idx >= static_cast<int>(kps.size()))
                continue;

            double obs_x = kps[static_cast<size_t>(idx)].pt.x;
            double obs_y = kps[static_cast<size_t>(idx)].pt.y;

            auto* cost = new ceres::AutoDiffCostFunction<GlobalBAProjectionFunctor, 2, 4, 3, 3>(
                new GlobalBAProjectionFunctor(obs_x, obs_y, fx, fy, cx, cy));

            problem.AddResidualBlock(cost, loss, kf_it->second->q, kf_it->second->t,
                                     mp_it->second->point);

            total_residuals++;
        }
    }

    if (total_residuals == 0) {
        return;
    }

    // Callback to check stop flag after each iteration
    class StopCallback : public ceres::IterationCallback {
    public:
        explicit StopCallback(std::atomic<bool>& flag) : flag_(flag) {}
        ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
            if (flag_)
                return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
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

    iterations_ = summary.iterations.size();
    final_cost_ = summary.final_cost;

    // Update keyframe poses
    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;

        auto it = kf_data_map.find(kf.get());
        if (it == kf_data_map.end())
            continue;

        const auto& data = it->second;
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

}  // namespace litevo::loop_closing

#else  // !LITEVO_HAS_CERES

namespace litevo::loop_closing {

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

}  // namespace litevo::loop_closing

#endif  // LITEVO_HAS_CERES
