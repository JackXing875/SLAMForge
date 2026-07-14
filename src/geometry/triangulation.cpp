// =============================================================================
// LiteVO triangulation — implementation
// =============================================================================

#include "litevo/geometry/triangulation.h"

#include <Eigen/SVD>

#include <cmath>
#include <limits>

#include "litevo/geometry/se3.h"

namespace litevo::geometry {

TriangulationResult TriangulatePoint(const Vec2& pt1, const Vec2& pt2, const Mat34& P1,
                                     const Mat34& P2, const Vec3& C1, const Vec3& C2,
                                     const TriangulationOptions& opts) {
    TriangulationResult result;

    // Build DLT matrix A (4x4)
    // A = [ pt1.x * P1(2,:) - P1(0,:) ]
    //     [ pt1.y * P1(2,:) - P1(1,:) ]
    //     [ pt2.x * P2(2,:) - P2(0,:) ]
    //     [ pt2.y * P2(2,:) - P2(1,:) ]
    Eigen::Matrix<Scalar, 4, 4> A;
    A.row(0) = pt1.x() * P1.row(2) - P1.row(0);
    A.row(1) = pt1.y() * P1.row(2) - P1.row(1);
    A.row(2) = pt2.x() * P2.row(2) - P2.row(0);
    A.row(3) = pt2.y() * P2.row(2) - P2.row(1);

    // Solve via SVD: the solution is the right singular vector of smallest singular value
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, 4, 4>> svd(A, Eigen::ComputeFullV);
    const Eigen::Matrix<Scalar, 4, 1> X = svd.matrixV().col(3);

    const Scalar w = X(3);
    if (std::abs(w) < 1e-10) {
        return result;  // invalid
    }

    result.point_w = Vec3(X(0) / w, X(1) / w, X(2) / w);

    // Numeric sanity check
    if (!std::isfinite(result.point_w.x()) || !std::isfinite(result.point_w.y()) ||
        !std::isfinite(result.point_w.z())) {
        result.valid = false;
        return result;
    }

    // Depth check (positive depth in both cameras)
    const Vec3 p_cam1 = P1.block<3, 3>(0, 0) * result.point_w + P1.col(3);
    const Vec3 p_cam2 = P2.block<3, 3>(0, 0) * result.point_w + P2.col(3);
    const Scalar depth1 = p_cam1.z();
    const Scalar depth2 = p_cam2.z();

    if (depth1 < opts.min_depth || depth2 < opts.min_depth) {
        return result;
    }
    if (depth1 > opts.max_depth || depth2 > opts.max_depth) {
        return result;
    }

    // Parallax check
    result.parallax_deg = ParallaxAngle(C1, C2, result.point_w);
    if (result.parallax_deg < opts.min_parallax_deg) {
        return result;
    }

    // Reprojection error
    result.reproj_error_1 = ReprojectionError(P1, result.point_w, pt1);
    result.reproj_error_2 = ReprojectionError(P2, result.point_w, pt2);

    if (result.reproj_error_1 > opts.max_reprojection_px ||
        result.reproj_error_2 > opts.max_reprojection_px) {
        return result;
    }

    result.valid = true;
    return result;
}

std::vector<TriangulationResult> TriangulatePoints(const std::vector<Vec2>& pts1,
                                                   const std::vector<Vec2>& pts2, const Mat34& P1,
                                                   const Mat34& P2, const Vec3& C1, const Vec3& C2,
                                                   const TriangulationOptions& opts) {
    std::vector<TriangulationResult> results;
    results.reserve(pts1.size());

    for (size_t i = 0; i < pts1.size(); ++i) {
        results.push_back(TriangulatePoint(pts1[i], pts2[i], P1, P2, C1, C2, opts));
    }

    return results;
}

double ReprojectionError(const Mat34& P, const Vec3& point_w, const Vec2& measured) {
    const Vec3 projected = P * Vec4(point_w.x(), point_w.y(), point_w.z(), 1.0);
    if (std::abs(projected.z()) < 1e-10) {
        return std::numeric_limits<double>::infinity();
    }
    const Vec2 predicted(projected.x() / projected.z(), projected.y() / projected.z());
    return (predicted - measured).norm();
}

double ParallaxAngle(const Vec3& C1, const Vec3& C2, const Vec3& point_w) {
    const Vec3 ray1 = (point_w - C1).normalized();
    const Vec3 ray2 = (point_w - C2).normalized();
    const double cos_angle = std::clamp(ray1.dot(ray2), -1.0, 1.0);
    return std::acos(cos_angle) * 180.0 / M_PI;
}

}  // namespace litevo::geometry
