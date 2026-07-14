// =============================================================================
// LiteVO SE(3) Lie algebra utilities
// =============================================================================
// Provides SE(3) exponential/logarithm maps, pose composition, and
// convenience constructors. Uses Eigen::Isometry3d internally.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "litevo/core/types.h"

namespace litevo::geometry {

// ── Exponential / logarithm maps ─────────────────────────────────────────────

/// @brief Compute the SE(3) exponential map: se(3) -> SE(3).
///
/// Maps a 6D twist vector (omega, upsilon) to a 4x4 transformation matrix.
/// omega = angular velocity (axis-angle), upsilon = linear velocity.
///
/// @param twist 6x1 vector: [vx, vy, vz, wx, wy, wz] where w = rotation, v = translation
/// @return SE(3) transformation matrix
[[nodiscard]] SE3 ExpSE3(const Vec6& twist);

/// @brief Compute the SE(3) logarithm map: SE(3) -> se(3).
///
/// Maps a 4x4 transformation matrix to a 6D twist vector.
///
/// @param T SE(3) transformation
/// @return 6x1 twist vector [vx, vy, vz, wx, wy, wz]
[[nodiscard]] Vec6 LogSE3(const SE3& T);

// ── Convenience constructors ─────────────────────────────────────────────────

/// @brief Create an SE(3) from a rotation matrix and translation vector.
[[nodiscard]] inline SE3 MakeSE3(const Mat3& R, const Vec3& t) {
    SE3 T = SE3::Identity();
    T.linear() = R;
    T.translation() = t;
    return T;
}

/// @brief Create an SE(3) from a quaternion and translation vector.
[[nodiscard]] inline SE3 MakeSE3(const Quaternion& q, const Vec3& t) {
    SE3 T = SE3::Identity();
    T.linear() = q.toRotationMatrix();
    T.translation() = t;
    return T;
}

/// @brief Create an SE(3) from an angle-axis rotation and translation.
[[nodiscard]] inline SE3 MakeSE3(const Vec3& axis_angle, const Vec3& t) {
    SE3 T = SE3::Identity();
    const Scalar angle = axis_angle.norm();
    if (angle > 1e-12) {
        T.linear() = Eigen::AngleAxis<Scalar>(angle, axis_angle.normalized()).toRotationMatrix();
    }
    T.translation() = t;
    return T;
}

// ── Pose utilities ───────────────────────────────────────────────────────────

/// @brief Extract the rotation matrix from an SE(3) transformation.
[[nodiscard]] inline Mat3 Rotation(const SE3& T) { return T.rotation(); }

/// @brief Extract the translation vector from an SE(3) transformation.
[[nodiscard]] inline Vec3 Translation(const SE3& T) { return T.translation(); }

/// @brief Extract the camera center (world position) from a camera pose.
///
/// For a world-to-camera pose T_cw, the camera center is -R^T * t.
[[nodiscard]] inline Vec3 CameraCenter(const SE3& T_cw) {
    return -T_cw.rotation().transpose() * T_cw.translation();
}

/// @brief Convert world-to-camera to camera-to-world.
[[nodiscard]] inline SE3 InvertPose(const SE3& T_cw) { return T_cw.inverse(); }

/// @brief Compute the relative transformation between two poses.
/// T_rel = T_wc2 * T_cw1 = T_wc2 * inv(T_wc1)
[[nodiscard]] inline SE3 RelativePose(const SE3& T_wc1, const SE3& T_wc2) {
    return T_wc1.inverse() * T_wc2;
}

// ── Transform utilities ──────────────────────────────────────────────────────

/// @brief Transform a 3D point from world to camera frame.
[[nodiscard]] inline Vec3 TransformPoint(const SE3& T_cw, const Vec3& p_w) {
    return T_cw * p_w;
}

/// @brief Transform a batch of 3D points.
inline void TransformPoints(const SE3& T_cw,
                            const std::vector<Vec3>& points_w,
                            std::vector<Vec3>& points_c) {
    points_c.resize(points_w.size());
    for (size_t i = 0; i < points_w.size(); ++i) {
        points_c[i] = T_cw * points_w[i];
    }
}

// ── Lie algebra adjoints ─────────────────────────────────────────────────────

/// @brief Compute the adjoint matrix of an SE(3) element.
[[nodiscard]] Mat6 AdjointSE3(const SE3& T);

// ── Utility functions ────────────────────────────────────────────────────────

/// @brief Compute the skew-symmetric matrix of a 3D vector.
///         [ 0  -z   y ]
/// v^  =   [ z   0  -x ]
///         [-y   x   0 ]
[[nodiscard]] Mat3 SkewSymmetric(const Vec3& v);

/// @brief Extract a 3D vector from a skew-symmetric matrix (inverse of SkewSymmetric).
[[nodiscard]] Vec3 VeeOperator(const Mat3& m);

}  // namespace litevo::geometry
