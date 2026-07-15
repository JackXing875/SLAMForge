// =============================================================================
// SLAMForge PnP solver — Perspective-n-Point pose estimation
// =============================================================================

#pragma once

#include <Eigen/Core>

#include <opencv2/core.hpp>

#include <vector>

#include "slamforge/core/types.h"

namespace slamforge::geometry {

/// @brief PnP estimation result.
struct PnPResult {
    SE3 T_cw = SE3::Identity();       ///< Estimated camera pose (world-to-camera)
    std::vector<int> inlier_indices;  ///< Inlier indices in the input arrays
    int num_inliers = 0;
    bool valid = false;
};

/// @brief PnP solver options.
struct PnPOptions {
    int max_iterations = 150;          ///< RANSAC max iterations
    double max_reproj_error = 4.0;     ///< Inlier reprojection threshold (pixels)
    double confidence = 0.999;         ///< RANSAC confidence
    bool use_extrinsic_guess = false;  ///< Use initial pose as starting point
};

/// @brief Solve PnP with RANSAC for robust outlier rejection.
///
/// Estimates the camera pose from 3D-2D correspondences using EPnP + RANSAC.
///
/// @param points_3d  3D points in world coordinates
/// @param points_2d  2D points in pixel coordinates
/// @param K          Camera intrinsic matrix (3x3)
/// @param options    Solver options
/// @return PnP result with pose and inlier info
[[nodiscard]] PnPResult SolvePnPRansac(const std::vector<cv::Point3f>& points_3d,
                                       const std::vector<cv::Point2f>& points_2d, const cv::Mat& K,
                                       const PnPOptions& options = {});

/// @brief Refine a PnP solution with nonlinear optimization (no RANSAC).
///
/// Uses only the inliers from SolvePnPRansac for refinement.
///
/// @param points_3d  Inlier 3D points
/// @param points_2d  Inlier 2D points
/// @param K          Camera intrinsic matrix
/// @param T_cw       Initial pose (will be updated in place)
/// @return True if refinement succeeded
[[nodiscard]] bool RefinePnP(const std::vector<cv::Point3f>& points_3d,
                             const std::vector<cv::Point2f>& points_2d, const cv::Mat& K,
                             SE3& T_cw);

/// @brief Convert OpenCV rvec/tvec to SLAMForge SE3.
[[nodiscard]] SE3 FromOpenCVRt(const cv::Mat& rvec, const cv::Mat& tvec);

/// @brief Convert SLAMForge SE3 to OpenCV rvec/tvec.
void ToOpenCVRt(const SE3& T_cw, cv::Mat& rvec, cv::Mat& tvec);

}  // namespace slamforge::geometry
