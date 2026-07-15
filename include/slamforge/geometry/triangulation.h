// =============================================================================
// SLAMForge triangulation — 3D point recovery from 2D correspondences
// =============================================================================

#pragma once

#include <vector>

#include "slamforge/core/types.h"

namespace slamforge::geometry {

/// @brief Quality metrics for a triangulated point.
struct TriangulationResult {
    Vec3 point_w;               ///< 3D point in world coordinates
    double parallax_deg = 0;    ///< Parallax angle in degrees
    double reproj_error_1 = 0;  ///< Reprojection error in camera 1 (pixels)
    double reproj_error_2 = 0;  ///< Reprojection error in camera 2 (pixels)
    bool valid = false;         ///< Whether the triangulation passed quality checks
};

/// @brief Triangulation options.
struct TriangulationOptions {
    double min_parallax_deg = 1.0;     ///< Minimum parallax angle (degrees)
    double max_reprojection_px = 4.0;  ///< Maximum reprojection error (pixels)
    double min_depth = 0.01;           ///< Minimum depth (meters, positive)
    double max_depth = 200.0;          ///< Maximum depth (meters)
};

/// @brief Triangulate a single 3D point from two 2D observations.
///
/// Uses the Direct Linear Transform (DLT) method with SVD.
///
/// @param pt1  2D point in camera 1 (normalized or pixel coordinates)
/// @param pt2  2D point in camera 2
/// @param P1   3x4 projection matrix for camera 1
/// @param P2   3x4 projection matrix for camera 2
/// @param C1   Camera center 1 (world coordinates)
/// @param C2   Camera center 2 (world coordinates)
/// @param opts Quality thresholds
/// @return Triangulation result with quality metrics
[[nodiscard]] TriangulationResult TriangulatePoint(const Vec2& pt1, const Vec2& pt2,
                                                   const Mat34& P1, const Mat34& P2, const Vec3& C1,
                                                   const Vec3& C2,
                                                   const TriangulationOptions& opts = {});

/// @brief Triangulate multiple points from two views.
///
/// @param pts1 2D points in camera 1
/// @param pts2 2D points in camera 2
/// @param P1   Projection matrix for camera 1
/// @param P2   Projection matrix for camera 2
/// @param C1   Camera center 1
/// @param C2   Camera center 2
/// @param opts Quality thresholds
/// @return Vector of triangulation results (same order as input)
[[nodiscard]] std::vector<TriangulationResult> TriangulatePoints(
    const std::vector<Vec2>& pts1, const std::vector<Vec2>& pts2, const Mat34& P1, const Mat34& P2,
    const Vec3& C1, const Vec3& C2, const TriangulationOptions& opts = {});

/// @brief Compute the reprojection error of a 3D point onto a 2D observation.
///
/// @param P       3x4 projection matrix
/// @param point_w 3D point in world coordinates
/// @param measured 2D measurement (pixels)
/// @return Reprojection error in pixels
[[nodiscard]] double ReprojectionError(const Mat34& P, const Vec3& point_w, const Vec2& measured);

/// @brief Compute the parallax angle between two rays from camera centers to a point.
///
/// @param C1 Camera center 1
/// @param C2 Camera center 2
/// @param point_w 3D point in world coordinates
/// @return Parallax angle in degrees
[[nodiscard]] double ParallaxAngle(const Vec3& C1, const Vec3& C2, const Vec3& point_w);

}  // namespace slamforge::geometry
