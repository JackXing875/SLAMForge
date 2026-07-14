// =============================================================================
// Sim(3) — 7-DoF similarity transform (rotation + translation + uniform scale)
// =============================================================================
// Monocular SLAM has scale ambiguity. Sim(3) captures this by adding a uniform
// scale factor to the rigid SE(3) transformation.
//
// A Sim(3) element maps a 3D point p via:  p' = s * R * p + t
// In homogeneous coordinates: [s*R  t; 0  1] * [p; 1]
//
// The Lie algebra sim(3) has 7 parameters:
//   v = [omega_x, omega_y, omega_z, upsilon_x, upsilon_y, upsilon_z, sigma]
// where omega = angular velocity (rotation), upsilon = linear velocity,
// sigma = scale velocity.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "litevo/core/types.h"

namespace litevo::geometry {

// ── Sim(3) type ────────────────────────────────────────────────────────────────

/// @brief A 7-DoF similarity transformation: p' = s * R * p + t.
struct Sim3 {
    Mat3 R = Mat3::Identity();  ///< Rotation matrix (SO(3))
    Vec3 t = Vec3::Zero();      ///< Translation vector
    Scalar s = 1.0;             ///< Uniform scale factor

    /// @brief Convert to 4x4 homogeneous matrix.
    /// @return [s*R  t; 0  1]
    [[nodiscard]] Mat4 Matrix() const;

    /// @brief Inverse transformation.
    [[nodiscard]] Sim3 Inverse() const;

    /// @brief Composition: this ∘ other.
    [[nodiscard]] Sim3 operator*(const Sim3& other) const;

    /// @brief Transform a 3D point.
    [[nodiscard]] Vec3 TransformPoint(const Vec3& p) const;

    /// @brief Identity element.
    [[nodiscard]] static Sim3 Identity() { return Sim3{}; }
};

/// @brief Alias for the 4x4 matrix representation of Sim(3).
using Sim3Mat = Mat4;

// ── Constructors ───────────────────────────────────────────────────────────────

/// @brief Create a Sim(3) from rotation, translation, and scale.
[[nodiscard]] Sim3 MakeSim3(const Mat3& R, const Vec3& t, Scalar s);

/// @brief Convert a 4x4 matrix to Sim3 (assumes valid Sim(3) matrix).
[[nodiscard]] Sim3 Sim3FromMatrix(const Mat4& M);

/// @brief Convert a Sim(3) to a pure SE(3) by dropping the scale factor.
/// Optionally apply a rotation correction (used after scale propagation).
[[nodiscard]] SE3 Sim3ToSE3(const Sim3& S);

// ── Exponential / logarithm maps ───────────────────────────────────────────────

/// @brief Exponential map: sim(3) → Sim(3).
/// Maps a 7D tangent vector to a Sim(3) transformation.
/// @param v 7x1 vector: [ωx, ωy, ωz, υx, υy, υz, σ]
///           ω = angular velocity, υ = linear velocity, σ = scale velocity
[[nodiscard]] Sim3 ExpSim3(const Eigen::Matrix<Scalar, 7, 1>& v);

/// @brief Logarithm map: Sim(3) → sim(3).
/// Maps a Sim(3) transformation to a 7D tangent vector.
[[nodiscard]] Eigen::Matrix<Scalar, 7, 1> LogSim3(const Sim3& S);

// ── Component extractors ───────────────────────────────────────────────────────

[[nodiscard]] inline Mat3 RotationFromSim3(const Sim3& S) {
    return S.R;
}
[[nodiscard]] inline Vec3 TranslationFromSim3(const Sim3& S) {
    return S.t;
}
[[nodiscard]] inline Scalar ScaleFromSim3(const Sim3& S) {
    return S.s;
}

// ── Sim(3) estimation from 3D correspondences ──────────────────────────────────

/// @brief Result of Sim(3) estimation from 3D-3D correspondences.
struct Sim3Estimate {
    Sim3 S;
    std::vector<int> inlier_indices;
    int num_inliers = 0;
    bool valid = false;
};

/// @brief Options for Sim(3) RANSAC estimation.
struct Sim3RansacOptions {
    int max_iterations = 300;
    double max_error_3d = 0.05;  ///< Max 3D distance for inlier
    double confidence = 0.999;
    int min_inliers = 15;
};

/// @brief Estimate Sim(3) from 3D-3D correspondences using RANSAC + Horn/Umeyama.
///
/// @param pts1  3D points in frame 1's coordinate system.
/// @param pts2  3D points in frame 2's coordinate system.
/// @param opts  RANSAC options.
/// @return Sim3 estimate with inlier info.
[[nodiscard]] Sim3Estimate EstimateSim3Ransac(const std::vector<Vec3>& pts1,
                                              const std::vector<Vec3>& pts2,
                                              const Sim3RansacOptions& opts = {});

/// @brief Closed-form Sim(3) estimation from 3 3D-3D correspondences.
/// Uses Horn's method (scale + rotation + translation from centroids).
/// @return Sim(3) mapping pts1 to pts2.
[[nodiscard]] Sim3 ComputeSim3FromThree(const Vec3& p1_a, const Vec3& p1_b, const Vec3& p1_c,
                                        const Vec3& p2_a, const Vec3& p2_b, const Vec3& p2_c);

/// @brief Umeyama's method: closed-form Sim(3) from N ≥ 3 correspondences.
[[nodiscard]] Sim3 UmeyamaSim3(const std::vector<Vec3>& pts1, const std::vector<Vec3>& pts2,
                               const std::vector<double>& weights = {});

}  // namespace litevo::geometry
