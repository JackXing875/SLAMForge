// =============================================================================
// LoopVerifier implementation — Sim(3) geometric verification
// =============================================================================

#include "slamforge/loop_closing/verifier.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <unordered_set>

#include "slamforge/core/camera.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"
#include "slamforge/geometry/pnp.h"
#include "slamforge/geometry/triangulation.h"
#include "slamforge/loop_closing/detector.h"

namespace slamforge::loop_closing {

LoopVerifier::LoopVerifier(const LoopVerifierConfig& config, const Camera* camera)
    : config_(config), camera_(camera) {}

VerificationResult LoopVerifier::Verify(std::shared_ptr<KeyFrame> current_kf,
                                        std::shared_ptr<KeyFrame> candidate_kf,
                                        LoopDetector& /*detector*/, Map& map) {
    VerificationResult result;

    if (!current_kf || !candidate_kf)
        return result;

    // Step 1: BoW-guided matching
    std::vector<std::pair<int, int>> matches;
    std::vector<std::pair<int, float>> word_weights_1, word_weights_2;
    int n_initial = SearchByBoW(current_kf, candidate_kf, matches, word_weights_1, word_weights_2);
    result.num_initial_matches = n_initial;

    if (n_initial < config_.min_boW_matches) {
        return result;
    }

    // A descriptor score alone is not a loop constraint.  Require a coherent
    // two-view epipolar model before consulting the much sparser landmark
    // associations.  This is the primary false-positive gate for repeated
    // building textures and replaces an unreliable exact-pixel map-point
    // reprojection requirement.
    std::vector<cv::Point2f> current_pixels;
    std::vector<cv::Point2f> candidate_pixels;
    current_pixels.reserve(matches.size());
    candidate_pixels.reserve(matches.size());
    const auto& current_keypoints = current_kf->KeyPointsUndistorted();
    const auto& candidate_keypoints = candidate_kf->KeyPointsUndistorted();
    for (const auto& [current_index, candidate_index] : matches) {
        current_pixels.push_back(current_keypoints[static_cast<size_t>(current_index)].pt);
        candidate_pixels.push_back(candidate_keypoints[static_cast<size_t>(candidate_index)].pt);
    }
    cv::Mat fundamental_mask;
    cv::findFundamentalMat(current_pixels, candidate_pixels, cv::FM_RANSAC, 2.0, 0.999,
                           fundamental_mask);
    if (fundamental_mask.empty()) {
        return result;
    }
    std::vector<std::pair<int, int>> geometric_matches;
    geometric_matches.reserve(matches.size());
    for (size_t index = 0; index < matches.size(); ++index) {
        if (fundamental_mask.at<uchar>(static_cast<int>(index)) != 0) {
            geometric_matches.push_back(matches[index]);
        }
    }
    matches = std::move(geometric_matches);
    result.num_geometric_matches = static_cast<int>(matches.size());
    if (result.num_geometric_matches < std::max(12, config_.min_inliers * 2)) {
        return result;
    }

    // Step 2: Build 3D-3D correspondences from matched features with map points
    std::vector<Vec3> pts_cur_w, pts_cand_w;
    std::vector<int> match_idx_cur, match_idx_cand;
    int n_3d = Build3DCorrespondences(current_kf, candidate_kf, matches, map, pts_cur_w, pts_cand_w,
                                      match_idx_cur, match_idx_cand);
    result.num_3d_correspondences = n_3d;

    if (n_3d < config_.min_inliers) {
        return result;
    }

    // Step 3: RANSAC Sim(3) estimation
    std::vector<double> target_radii;
    target_radii.reserve(pts_cand_w.size());
    Vec3 target_center = Vec3::Zero();
    for (const Vec3& point : pts_cand_w) {
        target_center += point;
    }
    target_center /= static_cast<double>(pts_cand_w.size());
    for (const Vec3& point : pts_cand_w) {
        target_radii.push_back((point - target_center).norm());
    }
    const auto median_it = target_radii.begin() + static_cast<std::vector<double>::difference_type>(
                                                      target_radii.size() / 2);
    std::nth_element(target_radii.begin(), median_it, target_radii.end());
    const double target_spread = *median_it;

    geometry::Sim3RansacOptions sim3_opts;
    sim3_opts.max_iterations = config_.ransac_iterations;
    // Monocular map scale is arbitrary.  Tie the 3D gate to the candidate
    // landmark cloud instead of assuming that every scene uses metres.
    sim3_opts.max_error_3d = std::clamp(target_spread * 0.20, config_.sim3_max_error_3d, 1.5);
    sim3_opts.min_inliers = config_.min_inliers;

    auto sim3_result = geometry::EstimateSim3Ransac(pts_cur_w, pts_cand_w, sim3_opts);
    result.num_sim3_inliers = sim3_result.num_inliers;
    result.estimated_scale = sim3_result.S.s;

    if (!sim3_result.valid || sim3_result.num_inliers < config_.min_inliers ||
        !std::isfinite(sim3_result.S.s) || sim3_result.S.s < 0.01 || sim3_result.S.s > 100.0 ||
        !sim3_result.S.R.allFinite() || !sim3_result.S.t.allFinite()) {
        return result;
    }

    // Step 4: Estimate the current camera directly in the older candidate
    // map with 3D-to-2D PnP. Approximate nearby depths are useful for scale,
    // but their 3D-to-3D rotation can be badly biased; PnP pins rotation to
    // actual image reprojection and prevents a sparse loop from flipping the
    // complete trajectory.
    geometry::Sim3 pose_consistent_sim3 = sim3_result.S;
    if (camera_) {
        std::vector<cv::Point3f> candidate_points;
        std::vector<cv::Point2f> current_observations;
        candidate_points.reserve(pts_cand_w.size());
        current_observations.reserve(pts_cand_w.size());
        for (size_t correspondence = 0; correspondence < pts_cand_w.size(); ++correspondence) {
            const int current_index = match_idx_cur[correspondence];
            if (current_index < 0 || current_index >= static_cast<int>(current_keypoints.size())) {
                continue;
            }
            const Vec3& point = pts_cand_w[correspondence];
            if (!point.allFinite()) {
                continue;
            }
            candidate_points.emplace_back(static_cast<float>(point.x()),
                                          static_cast<float>(point.y()),
                                          static_cast<float>(point.z()));
            current_observations.push_back(
                current_keypoints[static_cast<size_t>(current_index)].pt);
        }

        if (candidate_points.size() < static_cast<size_t>(config_.min_inliers)) {
            return result;
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << camera_->fx(), 0.0, camera_->cx(), 0.0,
                     camera_->fy(), camera_->cy(), 0.0, 0.0, 1.0);
        cv::Mat rvec, tvec, pnp_inliers;
        const cv::Mat distortion = cv::Mat::zeros(4, 1, CV_64F);
        const bool pnp_ok = cv::solvePnPRansac(
            candidate_points, current_observations, K, distortion, rvec, tvec, false,
            config_.ransac_iterations, static_cast<float>(config_.max_reproj_error), 0.999,
            pnp_inliers, cv::SOLVEPNP_EPNP);
        if (!pnp_ok || pnp_inliers.rows < config_.min_inliers) {
            return result;
        }

        std::vector<cv::Point3f> pnp_points;
        std::vector<cv::Point2f> pnp_observations;
        pnp_points.reserve(static_cast<size_t>(pnp_inliers.rows));
        pnp_observations.reserve(static_cast<size_t>(pnp_inliers.rows));
        for (int row = 0; row < pnp_inliers.rows; ++row) {
            const int index = pnp_inliers.at<int>(row);
            pnp_points.push_back(candidate_points[static_cast<size_t>(index)]);
            pnp_observations.push_back(current_observations[static_cast<size_t>(index)]);
        }
        cv::solvePnP(pnp_points, pnp_observations, K, distortion, rvec, tvec, true,
                     cv::SOLVEPNP_ITERATIVE);

        const SE3 desired_current_pose = geometry::FromOpenCVRt(rvec, tvec);
        const SE3 raw_current_pose = current_kf->Pose();
        const Mat3 correction_rotation =
            desired_current_pose.rotation().transpose() * raw_current_pose.rotation();
        const double rotation_cosine =
            std::clamp((correction_rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
        if (!desired_current_pose.matrix().allFinite() ||
            std::acos(rotation_cosine) > 90.0 * std::numbers::pi_v<double> / 180.0) {
            return result;
        }

        const Vec3 raw_center = current_kf->CameraCenter();
        const Vec3 desired_center =
            -desired_current_pose.rotation().transpose() * desired_current_pose.translation();
        pose_consistent_sim3.R = correction_rotation;
        pose_consistent_sim3.t =
            desired_center - pose_consistent_sim3.s * correction_rotation * raw_center;
        result.num_reprojection_inliers = pnp_inliers.rows;
    }

    // Step 5: Build final result
    result.valid = true;
    result.S_cw = pose_consistent_sim3;
    result.num_inliers = result.num_reprojection_inliers > 0
                             ? std::min(sim3_result.num_inliers, result.num_reprojection_inliers)
                             : sim3_result.num_inliers;
    result.matches = matches;

    // Collect matched map point IDs
    for (int idx : sim3_result.inlier_indices) {
        int cur_idx = match_idx_cur[static_cast<size_t>(idx)];
        int cand_idx = match_idx_cand[static_cast<size_t>(idx)];
        if (cur_idx >= 0 && cur_idx < current_kf->NumKeyPoints()) {
            result.matched_mps_cur.push_back(current_kf->MapPointIdAt(cur_idx));
        }
        if (cand_idx >= 0 && cand_idx < candidate_kf->NumKeyPoints()) {
            result.matched_mps_cand.push_back(candidate_kf->MapPointIdAt(cand_idx));
        }
    }

    return result;
}

int LoopVerifier::SearchByBoW(std::shared_ptr<KeyFrame> kf1, std::shared_ptr<KeyFrame> kf2,
                              std::vector<std::pair<int, int>>& matches,
                              std::vector<std::pair<int, float>>& /*word_weights_1*/,
                              std::vector<std::pair<int, float>>& /*word_weights_2*/) {
    if (!kf1 || !kf2)
        return 0;

    const auto& desc1 = kf1->Descriptors();
    const auto& desc2 = kf2->Descriptors();

    // Descriptor-based matching with ratio test
    cv::BFMatcher matcher(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(desc1, desc2, knn_matches, 2);

    matches.clear();
    matches.reserve(knn_matches.size());

    for (size_t i = 0; i < knn_matches.size(); ++i) {
        const auto& knn = knn_matches[i];
        if (knn.size() >= 2) {
            if (static_cast<double>(knn[0].distance) <
                config_.descriptor_ratio * static_cast<double>(knn[1].distance)) {
                matches.emplace_back(knn[0].queryIdx, knn[0].trainIdx);
            }
        } else if (knn.size() == 1) {
            matches.emplace_back(knn[0].queryIdx, knn[0].trainIdx);
        }
    }

    return static_cast<int>(matches.size());
}

int LoopVerifier::Build3DCorrespondences(std::shared_ptr<KeyFrame> kf_cur,
                                         std::shared_ptr<KeyFrame> kf_cand,
                                         const std::vector<std::pair<int, int>>& matches, Map& map,
                                         std::vector<Vec3>& pts_cur_w,
                                         std::vector<Vec3>& pts_cand_w,
                                         std::vector<int>& match_indices_cur,
                                         std::vector<int>& match_indices_cand) {
    pts_cur_w.clear();
    pts_cand_w.clear();
    match_indices_cur.clear();
    match_indices_cand.clear();

    struct AssociatedPoint {
        std::shared_ptr<MapPoint> map_point;
        Vec3 position = Vec3::Zero();
    };
    const auto find_map_point = [&map](const std::shared_ptr<KeyFrame>& keyframe,
                                       int feature_index) -> AssociatedPoint {
        auto map_point = map.GetMapPoint(keyframe->MapPointIdAt(feature_index));
        if (map_point && !map_point->IsBad()) {
            return {map_point, map_point->Position()};
        }

        const auto& keypoints = keyframe->KeyPointsUndistorted();
        if (feature_index < 0 || feature_index >= static_cast<int>(keypoints.size())) {
            return {};
        }
        const cv::Point2f center = keypoints[static_cast<size_t>(feature_index)].pt;
        const auto nearby = keyframe->GetFeaturesInArea(center.x, center.y, 5.0F);
        float best_distance_sq = 25.0F;
        std::shared_ptr<MapPoint> best;
        for (const int nearby_index : nearby) {
            auto nearby_point = map.GetMapPoint(keyframe->MapPointIdAt(nearby_index));
            if (!nearby_point || nearby_point->IsBad()) {
                continue;
            }
            const cv::Point2f delta = keypoints[static_cast<size_t>(nearby_index)].pt - center;
            const float distance_sq = delta.dot(delta);
            if (distance_sq < best_distance_sq) {
                best_distance_sq = distance_sq;
                best = std::move(nearby_point);
            }
        }
        if (!best) {
            return {};
        }

        // The nearby landmark supplies a stable local depth, while the
        // descriptor match supplies the exact image ray.  Back-projecting
        // that depth onto the matched ray is substantially less biased than
        // pretending the neighbor's 3D position belongs to another pixel.
        const Vec3 nearby_camera = keyframe->Pose() * best->Position();
        if (!nearby_camera.allFinite() || nearby_camera.z() <= 1e-6) {
            return {};
        }
        const Camera& camera = keyframe->GetCamera();
        const Vec3 ray =
            camera.Unproject(Vec2(static_cast<double>(center.x), static_cast<double>(center.y)));
        if (std::abs(ray.z()) <= 1e-9) {
            return {};
        }
        const Vec3 point_camera = ray * (nearby_camera.z() / ray.z());
        const Vec3 point_world = keyframe->Pose().inverse() * point_camera;
        return {best, point_world};
    };

    std::unordered_set<uint64_t> used_pairs;

    for (const auto& [idx_cur, idx_cand] : matches) {
        auto current_point = find_map_point(kf_cur, idx_cur);
        auto candidate_point = find_map_point(kf_cand, idx_cand);

        if (!current_point.map_point || !candidate_point.map_point)
            continue;
        if (current_point.map_point->IsBad() || candidate_point.map_point->IsBad())
            continue;
        const uint64_t pair_hash = current_point.map_point->Id().id * 0x9E3779B185EBCA87ULL ^
                                   candidate_point.map_point->Id().id;
        if (!used_pairs.insert(pair_hash).second) {
            continue;
        }

        pts_cur_w.push_back(current_point.position);
        pts_cand_w.push_back(candidate_point.position);
        match_indices_cur.push_back(idx_cur);
        match_indices_cand.push_back(idx_cand);
    }

    return static_cast<int>(pts_cur_w.size());
}

}  // namespace slamforge::loop_closing
