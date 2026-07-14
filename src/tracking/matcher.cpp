// =============================================================================
// FeatureMatcher implementation
// =============================================================================

#include "litevo/tracking/matcher.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "litevo/core/camera.h"
#include "litevo/core/frame.h"
#include "litevo/core/map_point.h"

namespace litevo::tracking {

// ── Descriptor-to-descriptor matching ──────────────────────────────────────

std::vector<std::pair<int, int>> FeatureMatcher::MatchByDescriptor(const cv::Mat& desc1,
                                                                   const cv::Mat& desc2,
                                                                   float ratio_threshold,
                                                                   bool cross_check) const {
    std::vector<std::pair<int, int>> matches;

    if (desc1.empty() || desc2.empty()) {
        return matches;
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(desc1, desc2, knn, 2);

    // First pass: ratio test
    for (const auto& knn_pair : knn) {
        if (knn_pair.size() < 2)
            continue;
        if (knn_pair[0].distance < ratio_threshold * knn_pair[1].distance) {
            matches.emplace_back(knn_pair[0].queryIdx, knn_pair[0].trainIdx);
        }
    }

    if (!cross_check) {
        return matches;
    }

    // Cross-check: find best match in reverse direction
    std::vector<std::vector<cv::DMatch>> knn_rev;
    matcher.knnMatch(desc2, desc1, knn_rev, 1);

    std::vector<int> best_rev(desc2.rows, -1);
    for (size_t i = 0; i < knn_rev.size(); ++i) {
        if (!knn_rev[i].empty()) {
            best_rev[i] = knn_rev[i][0].trainIdx;
        }
    }

    std::vector<std::pair<int, int>> cross_matches;
    for (const auto& [q_idx, t_idx] : matches) {
        if (q_idx >= 0 && q_idx < static_cast<int>(best_rev.size())) {
            (void)t_idx;
        }
        if (t_idx >= 0 && t_idx < static_cast<int>(best_rev.size())) {
            if (best_rev[static_cast<size_t>(t_idx)] == q_idx) {
                cross_matches.emplace_back(q_idx, t_idx);
            }
        }
    }

    return cross_matches;
}

// ── Search for initialization ──────────────────────────────────────────────

int FeatureMatcher::SearchForInitialization(const Frame& F1, const Frame& F2, int window_size,
                                            std::vector<int>& matches_12) const {
    matches_12.assign(F1.NumKeyPoints(), -1);

    if (F1.NumKeyPoints() == 0 || F2.NumKeyPoints() == 0) {
        return 0;
    }

    const auto& kps1 = F1.KeyPointsUndistorted();
    const auto& kps2 = F2.KeyPointsUndistorted();
    const auto& desc1 = F1.Descriptors();
    const auto& desc2 = F2.Descriptors();

    std::vector<std::pair<int, int>> raw_matches;

    for (int i = 0; i < F1.NumKeyPoints(); ++i) {
        float x = kps1[static_cast<size_t>(i)].pt.x;
        float y = kps1[static_cast<size_t>(i)].pt.y;

        auto candidates = F2.GetFeaturesInArea(x, y, static_cast<float>(window_size));

        int best_dist = 256;
        int second_best = 256;
        int best_idx = -1;

        for (int j : candidates) {
            int dist = DescriptorDistance(desc1.row(i), desc2.row(j));
            if (dist < best_dist) {
                second_best = best_dist;
                best_dist = dist;
                best_idx = j;
            } else if (dist < second_best) {
                second_best = dist;
            }
        }

        // Ratio test
        if (best_idx >= 0 && best_dist < 0.7f * second_best) {
            raw_matches.emplace_back(i, best_idx);
        }
    }

    // Filter by rotation histogram
    auto filtered = FilterByRotationHistogram(raw_matches, kps1, kps2);

    for (const auto& [i1, i2] : filtered) {
        matches_12[static_cast<size_t>(i1)] = i2;
    }

    return static_cast<int>(filtered.size());
}

// ── Search by projection ───────────────────────────────────────────────────

std::vector<std::pair<int, MapPointId>> FeatureMatcher::SearchByProjection(
    Frame& frame, const std::vector<std::shared_ptr<MapPoint>>& map_points, const Camera& camera,
    float max_dist_px, int max_desc_dist) const {
    std::vector<std::pair<int, MapPointId>> result;
    result.reserve(map_points.size());

    if (map_points.empty())
        return result;

    // Track which frame keypoints are already matched
    std::unordered_set<int> matched_kp_indices;

    const auto& desc_frame = frame.Descriptors();

    for (const auto& mp : map_points) {
        if (!mp || mp->IsBad())
            continue;

        Vec3 p_w = mp->Position();

        // Project to current frame
        Vec2 uv = camera.ProjectWorld(p_w, frame.Pose());

        // Check visibility
        if (!camera.IsInImage(uv, static_cast<int>(max_dist_px))) {
            continue;
        }

        // Get candidate keypoints in a window
        auto candidates = frame.GetFeaturesInArea(static_cast<float>(uv.x()),
                                                  static_cast<float>(uv.y()), max_dist_px);

        int best_dist = max_desc_dist + 1;
        int best_kp_idx = -1;

        for (int kp_idx : candidates) {
            // Skip already matched keypoints
            if (matched_kp_indices.count(kp_idx))
                continue;

            int dist = DescriptorDistance(mp->Descriptor(), desc_frame.row(kp_idx));
            if (dist < best_dist) {
                best_dist = dist;
                best_kp_idx = kp_idx;
            }
        }

        if (best_kp_idx >= 0 && best_dist <= max_desc_dist) {
            result.emplace_back(best_kp_idx, mp->Id());
            matched_kp_indices.insert(best_kp_idx);
        }
    }

    return result;
}

// ── Descriptor distance ────────────────────────────────────────────────────

int FeatureMatcher::DescriptorDistance(const cv::Mat& desc_a, const cv::Mat& desc_b) {
    if (desc_a.empty() || desc_b.empty()) {
        return 256;  // Maximum possible distance for 256-bit descriptor
    }
    return static_cast<int>(cv::norm(desc_a, desc_b, cv::NORM_HAMMING));
}

// ── Rotation histogram filter ──────────────────────────────────────────────

std::vector<std::pair<int, int>> FeatureMatcher::FilterByRotationHistogram(
    const std::vector<std::pair<int, int>>& matches, const std::vector<cv::KeyPoint>& kps1,
    const std::vector<cv::KeyPoint>& kps2) {
    if (matches.size() < 10) {
        return matches;  // Too few to filter meaningfully
    }

    constexpr int kHistoBins = 30;
    constexpr float kHistoFactor = 1.0f / (2.0f * static_cast<float>(M_PI));

    // Build rotation histogram
    std::vector<int> histo(kHistoBins, 0);
    for (const auto& [i1, i2] : matches) {
        float angle_diff =
            kps1[static_cast<size_t>(i1)].angle - kps2[static_cast<size_t>(i2)].angle;
        if (angle_diff < 0.0f)
            angle_diff += 360.0f;
        int bin = static_cast<int>(std::round(angle_diff * kHistoFactor * kHistoBins));
        if (bin >= kHistoBins)
            bin -= kHistoBins;
        if (bin < 0)
            bin = 0;
        histo[static_cast<size_t>(bin)]++;
    }

    // Find top 3 bins
    std::vector<int> histo_sorted = histo;
    std::sort(histo_sorted.begin(), histo_sorted.end(), std::greater<int>());
    int threshold_1 = histo_sorted[0];
    int threshold_2 = histo_sorted[1];
    int threshold_3 = histo_sorted[2];

    // Keep matches in top 3 bins
    std::vector<std::pair<int, int>> filtered;
    for (const auto& [i1, i2] : matches) {
        float angle_diff =
            kps1[static_cast<size_t>(i1)].angle - kps2[static_cast<size_t>(i2)].angle;
        if (angle_diff < 0.0f)
            angle_diff += 360.0f;
        int bin = static_cast<int>(std::round(angle_diff * kHistoFactor * kHistoBins));
        if (bin >= kHistoBins)
            bin -= kHistoBins;
        if (bin < 0)
            bin = 0;
        int count = histo[static_cast<size_t>(bin)];
        if (count >= threshold_3) {
            filtered.emplace_back(i1, i2);
        }
    }

    return filtered;
}

}  // namespace litevo::tracking
