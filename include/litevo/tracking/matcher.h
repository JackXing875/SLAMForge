// =============================================================================
// FeatureMatcher — descriptor matching and projection-based search
// =============================================================================

#pragma once

#include <opencv2/core/mat.hpp>

#include <memory>
#include <utility>
#include <vector>

#include "litevo/core/types.h"

namespace litevo {

class Camera;
class Frame;
class MapPoint;

namespace tracking {

/// @brief Feature matching utilities for tracking and initialization.
///
/// Provides:
///   - Descriptor-to-descriptor matching with Lowe's ratio test
///   - Projection-based matching (MapPoints → frame keypoints)
///   - Grid-based matching for two-view initialization
class FeatureMatcher {
public:
    FeatureMatcher() = default;

    /// @brief Match two sets of ORB descriptors using brute-force + ratio test.
    ///
    /// @param desc1  Query descriptors (N1 × 32, CV_8UC1).
    /// @param desc2  Train descriptors (N2 × 32, CV_8UC1).
    /// @param ratio_threshold  Lowe's ratio threshold (default 0.7).
    /// @param cross_check  If true, keep only mutual best matches.
    /// @return Pairs of (index_in_desc1, index_in_desc2).
    [[nodiscard]] std::vector<std::pair<int, int>> MatchByDescriptor(
        const cv::Mat& desc1, const cv::Mat& desc2,
        float ratio_threshold = 0.7f, bool cross_check = true) const;

    /// @brief Match features between two frames for monocular initialization.
    ///
    /// For each feature in F1, searches a window in F2's grid, matches by
    /// descriptor, and filters by rotation histogram consistency.
    ///
    /// @param F1           Reference frame.
    /// @param F2           Current frame.
    /// @param window_size  Search radius in pixels.
    /// @param matches_12   Output: per-reference-feature match index in F2 (-1 = no match).
    /// @return Number of matches found.
    int SearchForInitialization(
        const Frame& F1, const Frame& F2,
        int window_size, std::vector<int>& matches_12) const;

    /// @brief Project MapPoints into a frame and match by descriptor.
    ///
    /// For each MapPoint: projects into the frame, searches the feature
    /// grid in a window, and matches by Hamming distance.
    ///
    /// @param frame          Current frame (pose must be set).
    /// @param map_points     MapPoints to search for.
    /// @param camera         Camera model for projection.
    /// @param max_dist_px    Max search radius in pixels.
    /// @param max_desc_dist  Max Hamming distance for a valid match.
    /// @return Pairs of (keypoint_index_in_frame, MapPointId).
    [[nodiscard]] std::vector<std::pair<int, MapPointId>> SearchByProjection(
        Frame& frame,
        const std::vector<std::shared_ptr<MapPoint>>& map_points,
        const Camera& camera,
        float max_dist_px = 4.0f,
        int max_desc_dist = 50) const;

    /// @brief Compute Hamming distance between two ORB descriptors.
    [[nodiscard]] static int DescriptorDistance(
        const cv::Mat& desc_a, const cv::Mat& desc_b);

private:
    /// @brief Filter matches by rotation histogram consistency.
    /// Keeps matches whose orientation difference falls into one of the
    /// top 3 bins in the rotation histogram.
    static std::vector<std::pair<int, int>> FilterByRotationHistogram(
        const std::vector<std::pair<int, int>>& matches,
        const std::vector<cv::KeyPoint>& kps1,
        const std::vector<cv::KeyPoint>& kps2);
};

}  // namespace tracking
}  // namespace litevo
