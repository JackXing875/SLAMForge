// =============================================================================
// Bundle Adjustment implementation
// =============================================================================

#include "litevo/optimization/ba.h"

#ifdef LITEVO_HAS_CERES

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "litevo/core/camera.h"
#include "litevo/core/keyframe.h"
#include "litevo/core/map_point.h"
#include "litevo/geometry/se3.h"

namespace litevo::optimization {

// ── Ceres cost functor (must be at namespace scope, not local class) ───────

/// Reprojection error: (quaternion[4], translation[3], point[3]) → residual[2]
struct ProjectionFunctor {
    ProjectionFunctor(double ox, double oy, double fxx, double fyy, double cxx, double cyy)
        : ox_(ox), oy_(oy), fx_(fxx), fy_(fyy), cx_(cxx), cy_(cyy) {}

    template <typename T>
    bool operator()(const T* const q, const T* const t, const T* const pt, T* res) const {
        // Rotate point by quaternion
        T rotated[3];
        ceres::UnitQuaternionRotatePoint(q, pt, rotated);
        // Translate
        T pc[3] = {rotated[0] + t[0], rotated[1] + t[1], rotated[2] + t[2]};
        // Project
        T inv_z = T(1.0) / pc[2];
        T proj_x = T(fx_) * pc[0] * inv_z + T(cx_);
        T proj_y = T(fy_) * pc[1] * inv_z + T(cy_);
        // Residual
        res[0] = proj_x - T(ox_);
        res[1] = proj_y - T(oy_);
        return true;
    }

    double ox_, oy_, fx_, fy_, cx_, cy_;
};

// ── LocalBundleAdjuster ─────────────────────────────────────────────────────

LocalBundleAdjuster::LocalBundleAdjuster(const Camera& camera) : camera_(camera) {}

LocalBundleAdjuster::LocalBundleAdjuster(const Camera& camera, const BAOptions& opts)
    : camera_(camera), opts_(opts) {}

int LocalBundleAdjuster::Optimize(std::vector<KeyFrame*>& local_kfs,
                                  std::vector<KeyFrame*>& fixed_kfs,
                                  std::vector<MapPoint*>& map_points) {
    if (local_kfs.empty() || map_points.empty()) {
        return 0;
    }

    ceres::Problem problem;

    // Camera intrinsics
    double fx = camera_.fx();
    double fy = camera_.fy();
    double cx = camera_.cx();
    double cy = camera_.cy();

    // ── KF pose blocks: quaternion[4] + translation[3] ──────────────────

    struct KfData {
        double q[4] = {1, 0, 0, 0};  // w, x, y, z
        double t[3] = {0, 0, 0};     // x, y, z
        KeyFrame* kf;
    };

    std::unordered_map<KeyFrame*, std::unique_ptr<KfData>> kf_data_map;

    auto add_kf_pose = [&](KeyFrame* kf, bool is_fixed) {
        auto data = std::make_unique<KfData>();
        data->kf = kf;

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

        if (is_fixed) {
            problem.SetParameterBlockConstant(data->q);
            problem.SetParameterBlockConstant(data->t);
        }

        kf_data_map[kf] = std::move(data);
    };

    // Add local KFs (fix first for gauge freedom)
    for (auto* kf : local_kfs) {
        bool fix_first = (kf == local_kfs[0]);
        add_kf_pose(kf, fix_first);
    }

    // Add fixed KFs
    for (auto* kf : fixed_kfs) {
        if (kf_data_map.find(kf) == kf_data_map.end()) {
            add_kf_pose(kf, true);
        }
    }

    // ── MP blocks: point[3] ─────────────────────────────────────────────

    struct MpData {
        double point[3] = {0, 0, 0};
        MapPoint* mp;
    };
    std::unordered_map<MapPoint*, std::unique_ptr<MpData>> mp_data_map;

    for (auto* mp : map_points) {
        auto data = std::make_unique<MpData>();
        Vec3 pos = mp->Position();
        data->point[0] = pos.x();
        data->point[1] = pos.y();
        data->point[2] = pos.z();
        data->mp = mp;
        problem.AddParameterBlock(data->point, 3);
        mp_data_map[mp] = std::move(data);
    }

    // ── Add residuals ────────────────────────────────────────────────────

    std::vector<KeyFrame*> all_kfs = local_kfs;
    all_kfs.insert(all_kfs.end(), fixed_kfs.begin(), fixed_kfs.end());

    ceres::LossFunction* loss = opts_.use_huber ? new ceres::HuberLoss(opts_.huber_delta) : nullptr;

    int total_residuals = 0;

    for (auto* kf : all_kfs) {
        auto kf_it = kf_data_map.find(kf);
        if (kf_it == kf_data_map.end())
            continue;

        for (int idx = 0; idx < kf->NumKeyPoints(); ++idx) {
            MapPointId mp_id = kf->MapPointIdAt(idx);
            if (mp_id.id == 0)
                continue;

            // Find the MapPoint in our optimization set
            MapPoint* mp_raw = nullptr;
            for (auto* mp : map_points) {
                if (mp->Id() == mp_id) {
                    mp_raw = mp;
                    break;
                }
            }
            if (!mp_raw)
                continue;

            auto mp_it = mp_data_map.find(mp_raw);
            if (mp_it == mp_data_map.end())
                continue;

            const auto& kps = kf->KeyPoints();
            if (idx >= static_cast<int>(kps.size()))
                continue;

            double obs_x = kps[static_cast<size_t>(idx)].pt.x;
            double obs_y = kps[static_cast<size_t>(idx)].pt.y;

            auto* cost = new ceres::AutoDiffCostFunction<ProjectionFunctor, 2, 4, 3, 3>(
                new ProjectionFunctor(obs_x, obs_y, fx, fy, cx, cy));

            problem.AddResidualBlock(cost, loss,
                                     kf_it->second->q,       // quaternion
                                     kf_it->second->t,       // translation
                                     mp_it->second->point);  // 3D point

            total_residuals++;
        }
    }

    if (total_residuals == 0) {
        return 0;
    }

    // ── Solve ────────────────────────────────────────────────────────────

    ceres::Solver::Options solver_opts;
    solver_opts.linear_solver_type = ceres::DENSE_SCHUR;
    solver_opts.max_num_iterations = opts_.max_iterations;
    solver_opts.minimizer_progress_to_stdout = false;
    solver_opts.num_threads = 4;

    ceres::Solver::Summary summary;
    ceres::Solve(solver_opts, &problem, &summary);

    // ── Update poses ─────────────────────────────────────────────────────

    for (auto* kf : local_kfs) {
        auto it = kf_data_map.find(kf);
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

    // ── Update map points ────────────────────────────────────────────────

    for (auto* mp : map_points) {
        auto it = mp_data_map.find(mp);
        if (it == mp_data_map.end())
            continue;

        const auto& pt = it->second->point;
        mp->SetPosition(Vec3(pt[0], pt[1], pt[2]));
    }

    return 0;
}

}  // namespace litevo::optimization

#else  // !LITEVO_HAS_CERES

// Stub when Ceres is unavailable
#include "litevo/core/keyframe.h"
#include "litevo/core/map_point.h"

namespace litevo::optimization {

LocalBundleAdjuster::LocalBundleAdjuster(const Camera& camera) : camera_(camera) {}

LocalBundleAdjuster::LocalBundleAdjuster(const Camera& camera, const BAOptions& opts)
    : camera_(camera), opts_(opts) {}

int LocalBundleAdjuster::Optimize(std::vector<KeyFrame*>&, std::vector<KeyFrame*>&,
                                  std::vector<MapPoint*>&) {
    return 0;
}

}  // namespace litevo::optimization

#endif  // LITEVO_HAS_CERES
