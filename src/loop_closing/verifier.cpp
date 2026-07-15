// =============================================================================
// LoopVerifier implementation — Sim(3) geometric verification
// =============================================================================

#include "slamforge/loop_closing/verifier.h"

#include <algorithm>
#include <unordered_set>

#include "slamforge/core/camera.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"
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

    if (n_initial < config_.min_boW_matches) {
        return result;
    }

    // Step 2: Build 3D-3D correspondences from matched features with map points
    std::vector<Vec3> pts_cur_w, pts_cand_w;
    std::vector<int> match_idx_cur, match_idx_cand;
    int n_3d = Build3DCorrespondences(current_kf, candidate_kf, matches, map, pts_cur_w, pts_cand_w,
                                      match_idx_cur, match_idx_cand);

    if (n_3d < 12) {
        return result;
    }

    // Step 3: RANSAC Sim(3) estimation
    geometry::Sim3RansacOptions sim3_opts;
    sim3_opts.max_iterations = config_.ransac_iterations;
    sim3_opts.max_error_3d = config_.sim3_max_error_3d;
    sim3_opts.min_inliers = config_.min_inliers;

    auto sim3_result = geometry::EstimateSim3Ransac(pts_cur_w, pts_cand_w, sim3_opts);

    if (!sim3_result.valid || sim3_result.num_inliers < config_.min_inliers) {
        return result;
    }

    // Step 4: Reprojection check
    if (camera_) {
        int reproj_inliers = 0;
        for (int inlier_idx : sim3_result.inlier_indices) {
            Vec3 p_w = pts_cur_w[static_cast<size_t>(inlier_idx)];

            // Apply Sim3 correction to get point in corrected world frame
            Vec3 p_cand = sim3_result.S.TransformPoint(p_w);

            // Project into candidate KF
            Vec2 proj_cand = camera_->ProjectWorld(p_cand, candidate_kf->Pose());

            // Check against the matched keypoint in candidate KF
            int cand_idx = match_idx_cand[static_cast<size_t>(inlier_idx)];
            const auto& kps = candidate_kf->KeyPointsUndistorted();
            if (cand_idx >= 0 && cand_idx < static_cast<int>(kps.size())) {
                double dx =
                    proj_cand.x() - static_cast<double>(kps[static_cast<size_t>(cand_idx)].pt.x);
                double dy =
                    proj_cand.y() - static_cast<double>(kps[static_cast<size_t>(cand_idx)].pt.y);
                if (dx * dx + dy * dy < config_.max_reproj_error * config_.max_reproj_error) {
                    reproj_inliers++;
                }
            }
        }

        if (reproj_inliers < config_.min_inliers) {
            return result;
        }
    }

    // Step 5: Build final result
    result.valid = true;
    result.S_cw = sim3_result.S;
    result.num_inliers = sim3_result.num_inliers;
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
                config_.min_inlier_ratio * static_cast<double>(knn[1].distance)) {
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

    for (const auto& [idx_cur, idx_cand] : matches) {
        MapPointId mp_id_cur = kf_cur->MapPointIdAt(idx_cur);
        MapPointId mp_id_cand = kf_cand->MapPointIdAt(idx_cand);

        if (mp_id_cur.id == 0 || mp_id_cand.id == 0)
            continue;

        auto mp_cur = map.GetMapPoint(mp_id_cur);
        auto mp_cand = map.GetMapPoint(mp_id_cand);

        if (!mp_cur || !mp_cand)
            continue;
        if (mp_cur->IsBad() || mp_cand->IsBad())
            continue;

        pts_cur_w.push_back(mp_cur->Position());
        pts_cand_w.push_back(mp_cand->Position());
        match_indices_cur.push_back(idx_cur);
        match_indices_cand.push_back(idx_cand);
    }

    return static_cast<int>(pts_cur_w.size());
}

}  // namespace slamforge::loop_closing
