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

namespace {

constexpr int kRotationHistogramBins = 30;

/// Map an orientation difference in degrees to a cyclic histogram bin.
/// ORB keypoint angles are degrees, not radians; using 1/(2*pi) here made
/// values near 360 index far beyond the 30-element histogram and corrupted
/// heap memory during two-view initialization.
int RotationHistogramBin(float angle_diff_degrees) {
    float normalized = std::fmod(angle_diff_degrees, 360.0f);
    if (normalized < 0.0f) {
        normalized += 360.0f;
    }

    int bin = static_cast<int>(
        std::lround(normalized * static_cast<float>(kRotationHistogramBins) / 360.0f));
    bin %= kRotationHistogramBins;
    return bin < 0 ? bin + kRotationHistogramBins : bin;
}

}  // namespace

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

    std::vector<int> best_rev(static_cast<size_t>(desc2.rows), -1);
    for (size_t i = 0; i < knn_rev.size(); ++i) {
        if (!knn_rev[i].empty()) {
            best_rev[i] = knn_rev[i][0].trainIdx;
        }
    }

    std::vector<std::pair<int, int>> cross_matches;
    for (const auto& [q_idx, t_idx] : matches) {
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
    matches_12.assign(static_cast<size_t>(F1.NumKeyPoints()), -1);

    if (F1.NumKeyPoints() == 0 || F2.NumKeyPoints() == 0) {
        return 0;
    }

    const auto& kps1 = F1.KeyPointsUndistorted();
    const auto& kps2 = F2.KeyPointsUndistorted();
    const auto& desc1 = F1.Descriptors();
    const auto& desc2 = F2.Descriptors();

    // Keep the initialization correspondence set one-to-one.  Without this,
    // several reference features can select the same current feature and the
    // resulting map points overwrite one another's KeyFrame association.
    std::vector<int> best_ref_for_current(static_cast<size_t>(F2.NumKeyPoints()), -1);
    std::vector<int> best_dist_for_current(static_cast<size_t>(F2.NumKeyPoints()), 256);

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
        if (best_idx >= 0 && best_dist * 10 < 7 * second_best) {
            if (best_dist < best_dist_for_current[static_cast<size_t>(best_idx)]) {
                best_dist_for_current[static_cast<size_t>(best_idx)] = best_dist;
                best_ref_for_current[static_cast<size_t>(best_idx)] = i;
            }
        }
    }

    std::vector<std::pair<int, int>> raw_matches;
    raw_matches.reserve(static_cast<size_t>(F2.NumKeyPoints()));
    for (int current_idx = 0; current_idx < F2.NumKeyPoints(); ++current_idx) {
        const int reference_idx = best_ref_for_current[static_cast<size_t>(current_idx)];
        if (reference_idx >= 0) {
            raw_matches.emplace_back(reference_idx, current_idx);
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
        // Frame grids are built from undistorted keypoints.  Project to the
        // same coordinate system; using Camera::ProjectWorld here would
        // search at distorted pixels for cameras with lens distortion.
        Vec2 uv = camera.ProjectWorldUndistorted(p_w, frame.Pose());

        // Check visibility
        if (!camera.IsInImage(uv, static_cast<int>(max_dist_px))) {
            continue;
        }

        // The same frame can be searched more than once (a wider retry in
        // TrackWithMotionModel followed by local-map refinement).  Count a
        // landmark as visible at most once per frame, otherwise the quality
        // ratio is artificially driven down and valid recent points are
        // culled.  Tracker::ResetFound clears this per-frame flag.
        if (mp->MarkObserved()) {
            mp->IncreaseVisible();
        }

        // Get candidate keypoints in a window
        auto candidates = frame.GetFeaturesInArea(static_cast<float>(uv.x()),
                                                  static_cast<float>(uv.y()), max_dist_px);

        int best_dist = max_desc_dist + 1;
        int best_kp_idx = -1;

        for (int kp_idx : candidates) {
            // Skip already matched keypoints
            if (matched_kp_indices.contains(kp_idx) || frame.MapPointIdAt(kp_idx).id != 0)
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

    // Build rotation histogram
    std::vector<int> histo(kRotationHistogramBins, 0);
    for (const auto& [i1, i2] : matches) {
        float angle_diff =
            kps1[static_cast<size_t>(i1)].angle - kps2[static_cast<size_t>(i2)].angle;
        const int bin = RotationHistogramBin(angle_diff);
        histo[static_cast<size_t>(bin)]++;
    }

    // Find top 3 bins
    std::vector<int> histo_sorted = histo;
    std::sort(histo_sorted.begin(), histo_sorted.end(), std::greater<int>());
    const int threshold_3 = histo_sorted[2];

    // Keep matches in top 3 bins
    std::vector<std::pair<int, int>> filtered;
    for (const auto& [i1, i2] : matches) {
        float angle_diff =
            kps1[static_cast<size_t>(i1)].angle - kps2[static_cast<size_t>(i2)].angle;
        const int bin = RotationHistogramBin(angle_diff);
        int count = histo[static_cast<size_t>(bin)];
        if (count >= threshold_3) {
            filtered.emplace_back(i1, i2);
        }
    }

    return filtered;
}

}  // namespace litevo::tracking
