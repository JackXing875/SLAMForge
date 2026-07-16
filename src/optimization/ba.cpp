// =============================================================================
// Bundle Adjustment implementation
// =============================================================================

#include "slamforge/optimization/ba.h"

#ifdef SLAMFORGE_HAS_CERES

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "optimization/ceres_compat.h"
#include "slamforge/core/camera.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map_point.h"
#include "slamforge/geometry/se3.h"

namespace slamforge::optimization {

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
        // A raw 1/z becomes non-finite when an LM trial step crosses the
        // camera plane, poisoning the complete Schur system. This smooth
        // reciprocal is indistinguishable from 1/z at ordinary scene depth
        // while keeping rejected trial steps finite near z=0.
        T inv_z = pc[2] / (pc[2] * pc[2] + T(1e-8));
        T proj_x = T(fx_) * pc[0] * inv_z + T(cx_);
        T proj_y = T(fy_) * pc[1] * inv_z + T(cy_);
        // Residual
        res[0] = proj_x - T(ox_);
        res[1] = proj_y - T(oy_);
        return true;
    }

    double ox_, oy_, fx_, fy_, cx_, cy_;
};

/// Numerically negligible Tikhonov priors keep locally disconnected or
/// near-zero-parallax blocks positive definite. Reprojection terms still
/// dominate by many orders of magnitude, so these are not motion priors.
struct PointPriorFunctor {
    explicit PointPriorFunctor(const Vec3& point, double weight = 1e-4)
        : point_(point), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const point, T* residuals) const {
        residuals[0] = T(weight_) * (point[0] - T(point_.x()));
        residuals[1] = T(weight_) * (point[1] - T(point_.y()));
        residuals[2] = T(weight_) * (point[2] - T(point_.z()));
        return true;
    }

    Vec3 point_;
    double weight_;
};

struct PosePriorFunctor {
    PosePriorFunctor(const Eigen::Quaterniond& rotation, const Vec3& translation,
                     double weight = 1e-4)
        : rotation_(rotation), translation_(translation), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const q, const T* const t, T* residuals) const {
        const T reference_inverse[4] = {T(rotation_.w()), T(-rotation_.x()),
                                        T(-rotation_.y()), T(-rotation_.z())};
        T delta[4];
        ceres::QuaternionProduct(q, reference_inverse, delta);
        residuals[0] = T(2.0 * weight_) * delta[1];
        residuals[1] = T(2.0 * weight_) * delta[2];
        residuals[2] = T(2.0 * weight_) * delta[3];
        residuals[3] = T(weight_) * (t[0] - T(translation_.x()));
        residuals[4] = T(weight_) * (t[1] - T(translation_.y()));
        residuals[5] = T(weight_) * (t[2] - T(translation_.z()));
        return true;
    }

    Eigen::Quaterniond rotation_;
    Vec3 translation_;
    double weight_;
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

    // Anchor the oldest local keyframe, never the newest one.  Fixing the
    // current keyframe made every local BA bend the existing map around the
    // noisiest pose estimate.  If no external fixed keyframe exists, anchor a
    // second old pose as well to remove monocular scale gauge freedom.
    std::vector<KeyFrame*> anchor_candidates = local_kfs;
    std::sort(anchor_candidates.begin(), anchor_candidates.end(), [](const KeyFrame* lhs,
                                                                     const KeyFrame* rhs) {
        return lhs->Id().id < rhs->Id().id;
    });
    std::unordered_set<KeyFrame*> anchored_local_kfs;
    anchored_local_kfs.insert(anchor_candidates.front());
    if (fixed_kfs.empty() && anchor_candidates.size() > 1) {
        anchored_local_kfs.insert(anchor_candidates[1]);
    }

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

        detail::AddQuaternionParameterBlock(problem, data->q);
        problem.AddParameterBlock(data->t, 3);

        if (is_fixed) {
            problem.SetParameterBlockConstant(data->q);
            problem.SetParameterBlockConstant(data->t);
        }

        kf_data_map[kf] = std::move(data);
    };

    // Add local KFs with stable gauge anchors.
    for (auto* kf : local_kfs) {
        add_kf_pose(kf, anchored_local_kfs.contains(kf));
    }

    // Add fixed KFs
    for (auto* kf : fixed_kfs) {
        if (kf_data_map.find(kf) == kf_data_map.end()) {
            add_kf_pose(kf, true);
        }
    }

    // ── MP blocks: point[3] ─────────────────────────────────────────────

    std::vector<KeyFrame*> all_kfs = local_kfs;
    all_kfs.insert(all_kfs.end(), fixed_kfs.begin(), fixed_kfs.end());

    // Count observations inside this BA window before adding point blocks.
    // A point seen by fewer than two participating keyframes provides no
    // triangulation constraint and creates a free 3-D block in the Schur
    // system. Those blocks caused the dense Cholesky failures on long runs.
    std::unordered_map<uint64_t, int> observation_counts;
    for (const auto* kf : all_kfs) {
        if (!kf) {
            continue;
        }
        std::unordered_set<uint64_t> observed_in_keyframe;
        for (int index = 0; index < kf->NumKeyPoints(); ++index) {
            const MapPointId id = kf->MapPointIdAt(index);
            if (id.id != 0 && observed_in_keyframe.insert(id.id).second) {
                ++observation_counts[id.id];
            }
        }
    }

    struct MpData {
        double point[3] = {0, 0, 0};
        Vec3 original = Vec3::Zero();
        MapPoint* mp;
    };
    std::unordered_map<MapPoint*, std::unique_ptr<MpData>> mp_data_map;
    std::unordered_map<uint64_t, MapPoint*> map_point_by_id;

    for (auto* mp : map_points) {
        if (!mp || mp->IsBad() || observation_counts[mp->Id().id] < 2) {
            continue;
        }
        auto data = std::make_unique<MpData>();
        Vec3 pos = mp->Position();
        if (!pos.allFinite()) {
            continue;
        }
        data->point[0] = pos.x();
        data->point[1] = pos.y();
        data->point[2] = pos.z();
        data->original = pos;
        data->mp = mp;
        problem.AddParameterBlock(data->point, 3);
        map_point_by_id.emplace(mp->Id().id, mp);
        mp_data_map[mp] = std::move(data);
    }

    // ── Add residuals ────────────────────────────────────────────────────

    ceres::LossFunction* loss = opts_.use_huber ? new ceres::HuberLoss(opts_.huber_delta) : nullptr;

    int total_residuals = 0;
    std::unordered_map<KeyFrame*, int> residuals_per_keyframe;

    for (auto* kf : all_kfs) {
        auto kf_it = kf_data_map.find(kf);
        if (kf_it == kf_data_map.end())
            continue;

        for (int idx = 0; idx < kf->NumKeyPoints(); ++idx) {
            MapPointId mp_id = kf->MapPointIdAt(idx);
            if (mp_id.id == 0)
                continue;

            const auto raw_it = map_point_by_id.find(mp_id.id);
            if (raw_it == map_point_by_id.end())
                continue;
            MapPoint* mp_raw = raw_it->second;

            auto mp_it = mp_data_map.find(mp_raw);
            if (mp_it == mp_data_map.end())
                continue;

            const Vec3 point_camera = kf->Pose() * mp_raw->Position();
            if (!point_camera.allFinite() || point_camera.z() <= 1e-6) {
                continue;
            }

            const auto& kps = kf->KeyPointsUndistorted();
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
            ++residuals_per_keyframe[kf];
        }
    }

    if (total_residuals == 0) {
        return 0;
    }

    // Keep any camera with no surviving residuals constant so it cannot make
    // the normal equations rank deficient.
    for (auto* kf : local_kfs) {
        if (residuals_per_keyframe[kf] != 0) {
            const auto data = kf_data_map.find(kf);
            if (data != kf_data_map.end()) {
                const Eigen::Quaterniond original_rotation(data->second->q[0], data->second->q[1],
                                                           data->second->q[2], data->second->q[3]);
                const Vec3 original_translation(data->second->t[0], data->second->t[1],
                                                data->second->t[2]);
                auto* prior =
                    new ceres::AutoDiffCostFunction<PosePriorFunctor, 6, 4, 3>(
                        new PosePriorFunctor(original_rotation, original_translation));
                problem.AddResidualBlock(prior, nullptr, data->second->q, data->second->t);
            }
            continue;
        }
        const auto data = kf_data_map.find(kf);
        if (data != kf_data_map.end()) {
            problem.SetParameterBlockConstant(data->second->q);
            problem.SetParameterBlockConstant(data->second->t);
        }
    }

    for (const auto& [_, data] : mp_data_map) {
        auto* prior = new ceres::AutoDiffCostFunction<PointPriorFunctor, 3, 3>(
            new PointPriorFunctor(data->original));
        problem.AddResidualBlock(prior, nullptr, data->point);
    }

    // ── Solve ────────────────────────────────────────────────────────────

    ceres::Solver::Options solver_opts;
    solver_opts.linear_solver_type = ceres::DENSE_SCHUR;
    solver_opts.max_num_iterations = opts_.max_iterations;
    solver_opts.minimizer_progress_to_stdout = false;
    // Reproducible batch/video output is more valuable than a small local-BA
    // speedup. Multi-threaded floating-point reductions changed RANSAC inputs
    // several frames later and produced run-to-run trajectory variation.
    solver_opts.num_threads = 1;

    ceres::Solver::Summary summary;
    ceres::Solve(solver_opts, &problem, &summary);

    // Never publish a failed or divergent solution into the live map.  Ceres
    // can leave parameter blocks containing a partial unusable step after a
    // factorization failure.
    if (!summary.IsSolutionUsable() || !std::isfinite(summary.final_cost) ||
        (std::isfinite(summary.initial_cost) && summary.final_cost > summary.initial_cost * 1.01)) {
        return 0;
    }

    for (const auto& [_, data] : kf_data_map) {
        const Eigen::Map<const Eigen::Vector4d> quaternion(data->q);
        const Eigen::Map<const Vec3> translation(data->t);
        if (!quaternion.allFinite() || !translation.allFinite() || quaternion.norm() < 1e-8) {
            return 0;
        }
    }

    for (const auto& [_, data] : mp_data_map) {
        const Vec3 optimized(data->point[0], data->point[1], data->point[2]);
        const double original_scale = std::max(1.0, data->original.norm());
        if (!optimized.allFinite() || optimized.norm() > original_scale * 20.0 + 100.0 ||
            (optimized - data->original).norm() > original_scale * 10.0 + 10.0) {
            return 0;
        }
    }

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

}  // namespace slamforge::optimization

#else  // !SLAMFORGE_HAS_CERES

// Stub when Ceres is unavailable
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map_point.h"

namespace slamforge::optimization {

LocalBundleAdjuster::LocalBundleAdjuster(const Camera& camera) : camera_(camera) {}

LocalBundleAdjuster::LocalBundleAdjuster(const Camera& camera, const BAOptions& opts)
    : camera_(camera), opts_(opts) {}

int LocalBundleAdjuster::Optimize(std::vector<KeyFrame*>&, std::vector<KeyFrame*>&,
                                  std::vector<MapPoint*>&) {
    return 0;
}

}  // namespace slamforge::optimization

#endif  // SLAMFORGE_HAS_CERES
