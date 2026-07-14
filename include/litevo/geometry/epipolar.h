// =============================================================================
// LiteVO epipolar geometry — essential/fundamental matrix, pose recovery
// =============================================================================

#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "litevo/core/types.h"

namespace litevo::geometry {

/// @brief Result of epipolar pose estimation.
struct EpipolarResult {
    Mat3 R;                ///< Rotation matrix (3x3)
    Vec3 t;                ///< Translation vector (3x1)
    std::vector<int> inlier_indices;  ///< Indices of inlier correspondences
    int    num_inliers = 0;
    bool   valid = false;
};

/// @brief Epipolar geometry solver options.
struct EpipolarOptions {
    double confidence          = 0.9999; ///< RANSAC confidence level
    double epipolar_threshold  = 1.0;    ///< Inlier threshold in pixels
    int    min_points          = 20;     ///< Minimum points for estimation
};

/// @brief Estimate the essential matrix and recover relative pose.
///
/// Uses USAC_MAGSAC (state-of-the-art robust estimator) for outlier rejection,
/// then recovers R and t from the essential matrix.
///
/// @param pts1   2D points in camera 1 (pixel or normalized coordinates)
/// @param pts2   2D points in camera 2
/// @param K      Camera intrinsic matrix (3x3)
/// @param options Estimation options
/// @return EpipolarResult with R, t, and inlier information
[[nodiscard]] EpipolarResult EstimatePoseEssential(
    const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2,
    const cv::Mat& K,
    const EpipolarOptions& options = {});

/// @brief Recover camera pose from an essential matrix.
///
/// Decomposes E into 4 possible (R,t) pairs and selects the one
/// with the most points in front of both cameras (positive depth).
///
/// @param E      Essential matrix (3x3)
/// @param pts1   2D points in camera 1 (normalized)
/// @param pts2   2D points in camera 2 (normalized)
/// @param R_out  Output rotation matrix
/// @param t_out  Output translation vector
/// @param mask_inout  Output inlier mask (Nx1, CV_8UC1)
/// @return Number of points with positive depth
[[nodiscard]] int RecoverPose(const cv::Mat& E,
                              const std::vector<cv::Point2f>& pts1,
                              const std::vector<cv::Point2f>& pts2,
                              const cv::Mat& K,
                              cv::Mat& R_out,
                              cv::Mat& t_out,
                              cv::Mat& mask_inout);

/// @brief Compute the fundamental matrix from an essential matrix and camera intrinsics.
/// F = K^{-T} * E * K^{-1}
[[nodiscard]] Mat3 FundamentalFromEssential(const Mat3& E, const Mat3& K);

}  // namespace litevo::geometry
