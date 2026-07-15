// =============================================================================
// Sim(3) implementation — Lie group exponential, logarithm, estimation
// =============================================================================

#include "slamforge/geometry/sim3.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace slamforge::geometry {

namespace {

constexpr Scalar kSmallTheta = 1e-12;
constexpr Scalar kSmallSigma = 1e-12;

// ── Skew-symmetric matrix ─────────────────────────────────────────────────────

Mat3 Skew(const Vec3& v) {
    Mat3 m;
    m << Scalar(0), -v.z(), v.y(), v.z(), Scalar(0), -v.x(), -v.y(), v.x(), Scalar(0);
    return m;
}

// ── SO(3) exponential (Rodrigues formula) ─────────────────────────────────────

Mat3 ExpSO3(const Vec3& omega) {
    Scalar theta = omega.norm();
    if (theta < kSmallTheta) {
        Mat3 omega_hat = Skew(omega);
        return Mat3::Identity() + omega_hat + 0.5 * omega_hat * omega_hat;
    }
    Vec3 axis = omega / theta;
    Mat3 axis_hat = Skew(axis);
    return Mat3::Identity() + std::sin(theta) * axis_hat +
           (Scalar(1) - std::cos(theta)) * axis_hat * axis_hat;
}

// ── SO(3) logarithm ───────────────────────────────────────────────────────────

Vec3 LogSO3(const Mat3& R) {
    Scalar trace = R.trace();
    Scalar cos_theta = (trace - Scalar(1)) / Scalar(2);
    cos_theta = std::clamp(cos_theta, Scalar(-1), Scalar(1));
    Scalar theta = std::acos(cos_theta);

    Vec3 omega;
    if (theta < kSmallTheta) {
        omega = Vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) / Scalar(2);
    } else {
        Scalar denom = Scalar(2) * std::sin(theta);
        omega = Vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) * (theta / denom);
    }
    return omega;
}

}  // namespace

// ── Sim3 methods ───────────────────────────────────────────────────────────────

Mat4 Sim3::Matrix() const {
    Mat4 M = Mat4::Identity();
    M.block<3, 3>(0, 0) = s * R;
    M.block<3, 1>(0, 3) = t;
    return M;
}

Sim3 Sim3::Inverse() const {
    Sim3 inv;
    Scalar inv_s = Scalar(1) / s;
    inv.R = R.transpose();
    inv.t = -inv_s * (R.transpose() * t);
    inv.s = inv_s;
    return inv;
}

Sim3 Sim3::operator*(const Sim3& other) const {
    Sim3 result;
    result.R = R * other.R;
    result.t = s * (R * other.t) + t;
    result.s = s * other.s;
    return result;
}

Vec3 Sim3::TransformPoint(const Vec3& p) const {
    return s * (R * p) + t;
}

// ── Constructors ───────────────────────────────────────────────────────────────

Sim3 MakeSim3(const Mat3& R, const Vec3& t, Scalar s) {
    Sim3 S;
    S.R = R;
    S.t = t;
    S.s = s;
    return S;
}

Sim3 Sim3FromMatrix(const Mat4& M) {
    Sim3 S;
    // Extract scale as the norm of first column (since R is orthonormal)
    S.s = M.block<3, 1>(0, 0).norm();
    S.R = M.block<3, 3>(0, 0) / S.s;
    S.t = M.block<3, 1>(0, 3);
    return S;
}

SE3 Sim3ToSE3(const Sim3& S) {
    SE3 T = SE3::Identity();
    T.linear() = S.R;
    T.translation() = S.t;
    return T;
}

// ── Exponential / logarithm maps ───────────────────────────────────────────────

Sim3 ExpSim3(const Eigen::Matrix<Scalar, 7, 1>& v) {
    Vec3 omega = v.segment<3>(0);
    Vec3 upsilon = v.segment<3>(3);
    Scalar sigma = v(6);

    Sim3 S;
    S.R = ExpSO3(omega);
    Scalar theta = omega.norm();
    S.s = std::exp(sigma);

    // Compute translation Jacobian W and multiply: t = W * upsilon
    if (theta < kSmallTheta && std::abs(sigma) < kSmallSigma) {
        // Both small: t ≈ υ + 0.5*ω×υ + σ*υ/2
        S.t = upsilon + 0.5 * omega.cross(upsilon) + 0.5 * sigma * upsilon;
    } else if (theta < kSmallTheta) {
        // Small rotation only
        Scalar a = (S.s - Scalar(1)) / sigma;
        S.t = a * upsilon + 0.5 * omega.cross(upsilon);
    } else if (std::abs(sigma) < kSmallSigma) {
        // Small scale only → SE(3) case
        Scalar sin_t = std::sin(theta);
        Scalar cos_t = std::cos(theta);
        Vec3 axis = omega / theta;
        Mat3 A = Skew(axis);
        Scalar a = sin_t / theta;
        Scalar b = (Scalar(1) - cos_t) / theta;
        Mat3 W = Mat3::Identity() + b * A + (Scalar(1) - a) / theta * A * A;
        S.t = W * upsilon;
    } else {
        // General case: g2o-formula W matrix for Sim(3) exponential.
        // W = a*I + b*Omega + c*Omega^2  (Omega = skew(omega))
        // With A = skew(axis) = Omega/theta, A² = axis*axis^T - I:
        // W = (a - c*θ²)*I + (b*θ)*A + (c*θ²)*axis*axis^T
        Scalar sin_t = std::sin(theta);
        Scalar cos_t = std::cos(theta);
        Scalar denom = sigma * sigma + theta * theta;
        Scalar inv_denom = Scalar(1) / denom;

        // g2o coefficients (using Omega = skew(omega), not skew(axis))
        Scalar a = (S.s * sin_t * sigma + (Scalar(1) - S.s * cos_t) * theta) * inv_denom / theta;
        Scalar b = (S.s - Scalar(1)) / sigma -
                   ((S.s * cos_t - Scalar(1)) * sigma + S.s * sin_t * theta) * inv_denom;
        Scalar c = (Scalar(1) - S.s * cos_t + S.s * sin_t * sigma / theta) * inv_denom;

        // Convert to axis-skew formulation: W = w1*I + w2*A + w3*axis*axis^T
        Scalar w1 = a - c * theta * theta;
        Scalar w2 = b * theta;
        Scalar w3 = c * theta * theta;

        Vec3 axis = omega / theta;
        Mat3 A = Skew(axis);

        Mat3 W = w1 * Mat3::Identity() + w2 * A + w3 * (axis * axis.transpose());

        S.t = W * upsilon;
    }

    return S;
}

Eigen::Matrix<Scalar, 7, 1> LogSim3(const Sim3& S) {
    Vec3 omega = LogSO3(S.R);
    Scalar sigma = std::log(S.s);

    Scalar theta = omega.norm();

    Eigen::Matrix<Scalar, 7, 1> v;

    if (theta < kSmallTheta) {
        // Small rotation
        if (std::abs(sigma) < kSmallSigma) {
            // Both small: v ≈ [ω; t]
            v << omega, S.t, sigma;
        } else {
            // Small rotation, non-zero scale
            Scalar a = (S.s - Scalar(1)) / sigma;
            Vec3 upsilon = S.t / a;
            v << omega, upsilon, sigma;
        }
    } else if (std::abs(sigma) < kSmallSigma) {
        // Non-zero rotation, small scale → SE(3) case
        Vec3 axis = omega / theta;
        Mat3 A = Skew(axis);

        Scalar sin_t = std::sin(theta);
        Scalar cos_t = std::cos(theta);
        Scalar a = sin_t / theta;
        Scalar b = (Scalar(1) - cos_t) / theta;

        // W⁻¹ = I - θ/2*A + (1 - a/(2b))/θ² * A²
        Scalar c = (Scalar(1) - a / (Scalar(2) * b)) / (theta * theta);
        Mat3 W_inv = Mat3::Identity() - 0.5 * A + c * A * A;

        Vec3 upsilon = W_inv * S.t;
        v << omega, upsilon, sigma;
    } else {
        // General case: invert W using g2o-formula coefficients
        Scalar sin_t = std::sin(theta);
        Scalar cos_t = std::cos(theta);
        Scalar denom = sigma * sigma + theta * theta;
        Scalar inv_denom = Scalar(1) / denom;

        // g2o coefficients (using Omega = skew(omega), not skew(axis))
        Scalar a = (S.s * sin_t * sigma + (Scalar(1) - S.s * cos_t) * theta) * inv_denom / theta;
        Scalar b = (S.s - Scalar(1)) / sigma -
                   ((S.s * cos_t - Scalar(1)) * sigma + S.s * sin_t * theta) * inv_denom;
        Scalar c = (Scalar(1) - S.s * cos_t + S.s * sin_t * sigma / theta) * inv_denom;

        // Convert to axis-skew formulation: W = w1*I + w2*A + w3*axis*axis^T
        Scalar w1 = a - c * theta * theta;
        Scalar w2 = b * theta;
        Scalar w3 = c * theta * theta;

        Vec3 axis = omega / theta;
        Mat3 A = Skew(axis);

        Mat3 W = w1 * Mat3::Identity() + w2 * A + w3 * (axis * axis.transpose());

        Vec3 upsilon = W.inverse() * S.t;
        v << omega, upsilon, sigma;
    }

    return v;
}

// ── Closed-form Sim(3) estimation ──────────────────────────────────────────────

Sim3 UmeyamaSim3(const std::vector<Vec3>& pts1, const std::vector<Vec3>& pts2,
                 const std::vector<double>& weights) {
    if (pts1.size() < 3 || pts1.size() != pts2.size()) {
        return Sim3::Identity();
    }

    const size_t N = pts1.size();
    const bool weighted = (weights.size() == N);

    // Compute weighted centroids
    Vec3 centroid1 = Vec3::Zero();
    Vec3 centroid2 = Vec3::Zero();
    double total_weight = 0;

    for (size_t i = 0; i < N; ++i) {
        double w = weighted ? weights[i] : 1.0;
        centroid1 += w * pts1[i];
        centroid2 += w * pts2[i];
        total_weight += w;
    }
    centroid1 /= total_weight;
    centroid2 /= total_weight;

    // Compute covariance matrix
    Mat3 H = Mat3::Zero();
    double sigma1 = 0;  // sum of squared norms of pts1

    for (size_t i = 0; i < N; ++i) {
        double w = weighted ? weights[i] : 1.0;
        Vec3 d1 = pts1[i] - centroid1;
        Vec3 d2 = pts2[i] - centroid2;
        H += w * d2 * d1.transpose();
        sigma1 += w * d1.squaredNorm();
    }
    // Note: scale formula uses un-normalized sums; both H and sigma1
    // have the same weighting, so the ratio s = trace(R*H)/sigma1 is correct.

    // SVD for rotation
    Eigen::JacobiSVD<Mat3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 U = svd.matrixU();
    Mat3 V = svd.matrixV();

    Mat3 R = U * V.transpose();
    if (R.determinant() < 0) {
        // Handle reflection case
        Mat3 U_fixed = U;
        U_fixed.col(2) = -U.col(2);
        R = U_fixed * V.transpose();
    }

    // Scale
    Scalar s = (sigma1 > kSmallTheta) ? (R * H.transpose()).trace() / sigma1 : Scalar(1);

    // Translation
    Vec3 t = centroid2 - s * R * centroid1;

    return MakeSim3(R, t, s);
}

Sim3 ComputeSim3FromThree(const Vec3& p1_a, const Vec3& p1_b, const Vec3& p1_c, const Vec3& p2_a,
                          const Vec3& p2_b, const Vec3& p2_c) {
    std::vector<Vec3> pts1 = {p1_a, p1_b, p1_c};
    std::vector<Vec3> pts2 = {p2_a, p2_b, p2_c};
    return UmeyamaSim3(pts1, pts2);
}

// ── RANSAC for Sim(3) ──────────────────────────────────────────────────────────

Sim3Estimate EstimateSim3Ransac(const std::vector<Vec3>& pts1, const std::vector<Vec3>& pts2,
                                const Sim3RansacOptions& opts) {
    Sim3Estimate result;

    if (pts1.size() < 3 || pts1.size() != pts2.size()) {
        return result;
    }

    const size_t N = pts1.size();
    const double max_err_sq = opts.max_error_3d * opts.max_error_3d;
    const double log_1_minus_conf = std::log(1.0 - opts.confidence);

    std::mt19937 rng(42);  // Fixed seed for reproducibility
    std::uniform_int_distribution<size_t> dist(0, N - 1);

    int best_inliers = 0;
    Sim3 best_S;

    // Adaptive RANSAC
    int max_iters = opts.max_iterations;
    int iter = 0;

    while (iter < max_iters) {
        // Pick 3 random points
        size_t i1 = dist(rng);
        size_t i2 = dist(rng);
        size_t i3 = dist(rng);
        if (i1 == i2 || i1 == i3 || i2 == i3) {
            ++iter;
            continue;
        }

        // Compute Sim3 from these 3 points
        Sim3 S = ComputeSim3FromThree(pts1[i1], pts1[i2], pts1[i3], pts2[i1], pts2[i2], pts2[i3]);

        // Count inliers
        std::vector<int> inlier_indices;
        for (size_t i = 0; i < N; ++i) {
            Vec3 p_proj = S.TransformPoint(pts1[i]);
            double err_sq = (p_proj - pts2[i]).squaredNorm();
            if (err_sq < max_err_sq) {
                inlier_indices.push_back(static_cast<int>(i));
            }
        }

        if (static_cast<int>(inlier_indices.size()) > best_inliers) {
            best_inliers = static_cast<int>(inlier_indices.size());
            best_S = S;
            result.inlier_indices = std::move(inlier_indices);

            // Update max iterations adaptively
            double inlier_ratio = static_cast<double>(best_inliers) / static_cast<double>(N);
            if (inlier_ratio > 0) {
                max_iters =
                    static_cast<int>(log_1_minus_conf / std::log(1.0 - std::pow(inlier_ratio, 3)));
                max_iters = std::min(max_iters, opts.max_iterations);
            }
        }

        ++iter;
    }

    // Final refinement using all inliers
    if (best_inliers >= opts.min_inliers) {
        std::vector<Vec3> inlier_pts1, inlier_pts2;
        inlier_pts1.reserve(result.inlier_indices.size());
        inlier_pts2.reserve(result.inlier_indices.size());
        for (int idx : result.inlier_indices) {
            inlier_pts1.push_back(pts1[static_cast<size_t>(idx)]);
            inlier_pts2.push_back(pts2[static_cast<size_t>(idx)]);
        }
        result.S = UmeyamaSim3(inlier_pts1, inlier_pts2);
        result.num_inliers = best_inliers;
        result.valid = true;
    }

    return result;
}

}  // namespace slamforge::geometry
