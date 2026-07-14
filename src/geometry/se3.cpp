// =============================================================================
// LiteVO SE(3) Lie algebra — implementation
// =============================================================================

#include "litevo/geometry/se3.h"

#include <cmath>
#include <limits>

namespace litevo::geometry {

namespace {

constexpr Scalar kEpsilon = 1e-12;

/// @brief Rodrigues' formula: compute SO(3) exponential map.
Mat3 ExpSO3(const Vec3& omega) {
    const Scalar theta = omega.norm();
    if (theta < kEpsilon) {
        // Small angle approximation: exp(omega^) ≈ I + omega^ + 0.5 * omega^2
        Mat3 omega_hat;
        omega_hat << Scalar(0), -omega.z(), omega.y(), omega.z(), Scalar(0), -omega.x(), -omega.y(),
            omega.x(), Scalar(0);
        return Mat3::Identity() + omega_hat + Scalar(0.5) * omega_hat * omega_hat;
    }

    const Vec3 axis = omega / theta;
    Mat3 axis_hat;
    axis_hat << Scalar(0), -axis.z(), axis.y(), axis.z(), Scalar(0), -axis.x(), -axis.y(), axis.x(),
        Scalar(0);

    return Mat3::Identity() + std::sin(theta) * axis_hat +
           (Scalar(1) - std::cos(theta)) * axis_hat * axis_hat;
}

/// @brief Compute the left-Jacobian of SO(3).
Mat3 LeftJacobianSO3(const Vec3& omega) {
    const Scalar theta = omega.norm();
    if (theta < kEpsilon) {
        return Mat3::Identity() + Scalar(0.5) * SkewSymmetric(omega);
    }

    const Vec3 axis = omega / theta;
    const Mat3 axis_hat = SkewSymmetric(axis);
    const Scalar sin_theta = std::sin(theta);
    const Scalar cos_theta = std::cos(theta);

    return Mat3::Identity() + (Scalar(1) - cos_theta) / theta * axis_hat +
           (theta - sin_theta) / theta * axis_hat * axis_hat;
}

}  // namespace

Mat3 SkewSymmetric(const Vec3& v) {
    Mat3 m;
    m << Scalar(0), -v.z(), v.y(), v.z(), Scalar(0), -v.x(), -v.y(), v.x(), Scalar(0);
    return m;
}

SE3 ExpSE3(const Vec6& twist) {
    const Vec3 upsilon = twist.head<3>();  // translation part
    const Vec3 omega = twist.tail<3>();    // rotation part

    const Mat3 R = ExpSO3(omega);
    const Mat3 V = LeftJacobianSO3(omega);
    const Vec3 t = V * upsilon;

    return MakeSE3(R, t);
}

Vec6 LogSE3(const SE3& T) {
    const Mat3 R = T.rotation();
    const Vec3 t = T.translation();

    // Compute the rotation vector (log of SO(3))
    const Scalar trace = R.trace();
    const Scalar cos_theta = (trace - Scalar(1)) / Scalar(2);
    const Scalar theta = std::acos(std::clamp(cos_theta, Scalar(-1), Scalar(1)));

    Vec3 omega;
    if (theta < kEpsilon) {
        // Small angle: omega^ ≈ (R - R^T) / 2
        omega = VeeOperator((R - R.transpose()) / Scalar(2));
    } else {
        omega = theta / (Scalar(2) * std::sin(theta)) * VeeOperator(R - R.transpose());
    }

    // Compute the translation part of the twist
    const Mat3 V_inv = LeftJacobianSO3(-omega);  // V^{-1}(omega) = V(-omega) for small angles
    // Actually V^{-1} is more complex; use approximation for now
    // V^{-1} ≈ I - 0.5*omega^ + (1/theta^2 - (1+cos(theta))/(2*theta*sin(theta))) * omega^2
    Mat3 omega_hat = SkewSymmetric(omega);
    Mat3 V_inv_approx;
    if (theta < kEpsilon) {
        V_inv_approx =
            Mat3::Identity() - Scalar(0.5) * omega_hat + Scalar(1.0 / 12.0) * omega_hat * omega_hat;
    } else {
        const Scalar a = std::sin(theta) / theta;
        const Scalar b = (Scalar(1) - cos_theta) / (theta * theta);
        const Scalar c = (Scalar(1) - a / (Scalar(2) * b)) / (theta * theta);
        V_inv_approx = Mat3::Identity() - Scalar(0.5) * omega_hat + c * omega_hat * omega_hat;
    }

    const Vec3 upsilon = V_inv_approx * t;

    Vec6 twist;
    twist << upsilon, omega;
    return twist;
}

Vec3 VeeOperator(const Mat3& m) {
    return Vec3(m(2, 1), m(0, 2), m(1, 0));
}

Mat6 AdjointSE3(const SE3& T) {
    const Mat3 R = T.rotation();
    const Mat3 t_hat = SkewSymmetric(T.translation());

    Mat6 Adj = Mat6::Zero();
    Adj.block<3, 3>(0, 0) = R;
    Adj.block<3, 3>(0, 3) = t_hat * R;
    Adj.block<3, 3>(3, 3) = R;
    return Adj;
}

}  // namespace litevo::geometry
