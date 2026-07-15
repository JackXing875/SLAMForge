// =============================================================================
// Sim(3) unit tests — Lie group operations and estimation
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "slamforge/geometry/se3.h"
#include "slamforge/geometry/sim3.h"

using namespace slamforge;
using namespace slamforge::geometry;

// ── Construction and basic operations ─────────────────────────────────────────

TEST(Sim3Test, Identity) {
    Sim3 I = Sim3::Identity();
    EXPECT_NEAR(I.s, 1.0, 1e-12);
    EXPECT_NEAR(I.R.trace(), 3.0, 1e-12);
    EXPECT_NEAR(I.t.norm(), 0.0, 1e-12);
}

TEST(Sim3Test, MakeSim3) {
    Mat3 R = Mat3::Identity();
    Vec3 t(1, 2, 3);
    Scalar s = 2.0;
    Sim3 S = MakeSim3(R, t, s);

    EXPECT_NEAR(S.s, 2.0, 1e-12);
    EXPECT_NEAR(S.t.x(), 1.0, 1e-12);
    EXPECT_NEAR(S.t.y(), 2.0, 1e-12);
    EXPECT_NEAR(S.t.z(), 3.0, 1e-12);
}

TEST(Sim3Test, MatrixConversion) {
    Mat3 R = Eigen::AngleAxisd(0.5, Vec3::UnitZ()).toRotationMatrix();
    Vec3 t(1, 2, 3);
    Scalar s = 2.0;
    Sim3 S = MakeSim3(R, t, s);

    Mat4 M = S.Matrix();
    // Top-left 3x3 should be s*R
    Mat3 sR = M.block<3, 3>(0, 0);
    EXPECT_NEAR((sR - s * R).norm(), 0.0, 1e-12);
    // Translation
    EXPECT_NEAR(M(0, 3), 1.0, 1e-12);
    EXPECT_NEAR(M(1, 3), 2.0, 1e-12);
    EXPECT_NEAR(M(2, 3), 3.0, 1e-12);
}

TEST(Sim3Test, TransformPoint) {
    Mat3 R = Eigen::AngleAxisd(std::numbers::pi_v<double> / 2.0, Vec3::UnitZ()).toRotationMatrix();
    Vec3 t(0, 0, 0);
    Scalar s = 2.0;
    Sim3 S = MakeSim3(R, t, s);

    Vec3 p(1, 0, 0);
    Vec3 expected(0, 2, 0);  // Rotated 90° about Z, then scaled by 2
    Vec3 result = S.TransformPoint(p);
    EXPECT_NEAR((result - expected).norm(), 0.0, 1e-10);
}

TEST(Sim3Test, Inverse) {
    Mat3 R = Eigen::AngleAxisd(0.3, Vec3(1, 0, 1).normalized()).toRotationMatrix();
    Vec3 t(1, 2, 3);
    Scalar s = 1.5;
    Sim3 S = MakeSim3(R, t, s);
    Sim3 inv = S.Inverse();

    // S^{-1} * S should be identity
    Sim3 comp = inv * S;
    EXPECT_NEAR(comp.R.trace(), 3.0, 1e-10);
    EXPECT_NEAR(comp.t.norm(), 0.0, 1e-10);
    EXPECT_NEAR(comp.s, 1.0, 1e-10);
}

TEST(Sim3Test, Composition) {
    Sim3 S1 = MakeSim3(Mat3::Identity(), Vec3(1, 0, 0), 2.0);
    Sim3 S2 = MakeSim3(Mat3::Identity(), Vec3(0, 1, 0), 3.0);

    Sim3 comp = S1 * S2;
    // s = s1 * s2 = 6
    EXPECT_NEAR(comp.s, 6.0, 1e-12);
    // t = s1 * R1 * t2 + t1 = 2 * I * (0,1,0) + (1,0,0) = (1,2,0)
    EXPECT_NEAR(comp.t.x(), 1.0, 1e-12);
    EXPECT_NEAR(comp.t.y(), 2.0, 1e-12);
    EXPECT_NEAR(comp.t.z(), 0.0, 1e-12);
}

// ── Exponential and logarithm maps ────────────────────────────────────────────

TEST(Sim3Test, ExpLogRoundTrip) {
    // Test with a small 7D twist
    Eigen::Matrix<Scalar, 7, 1> v;
    v << 0.1, 0.05, -0.03,  // omega (rotation)
        0.2, -0.1, 0.15,    // upsilon (translation)
        0.05;               // sigma (scale)

    Sim3 S = ExpSim3(v);
    Eigen::Matrix<Scalar, 7, 1> v2 = LogSim3(S);

    // The recovered twist should be close to original
    EXPECT_NEAR((v - v2).norm(), 0.0, 1e-4);
}

TEST(Sim3Test, ExpLogRoundTripNoScale) {
    // Pure SE(3) case (sigma = 0)
    Eigen::Matrix<Scalar, 7, 1> v;
    v << 0.1, 0.2, 0.3,  // omega
        1.0, 2.0, 3.0,   // upsilon
        0.0;             // sigma = 0

    Sim3 S = ExpSim3(v);
    EXPECT_NEAR(S.s, 1.0, 1e-10);

    Eigen::Matrix<Scalar, 7, 1> v2 = LogSim3(S);
    EXPECT_NEAR((v - v2).norm(), 0.0, 1e-3);
}

TEST(Sim3Test, ExpSim3SmallAngle) {
    // Very small rotation to test Taylor expansion path
    Eigen::Matrix<Scalar, 7, 1> v;
    v << 1e-6, 2e-6, 3e-6,  // tiny omega
        0.1, 0.2, 0.3,      // upsilon
        0.01;               // sigma

    Sim3 S = ExpSim3(v);
    EXPECT_NEAR(S.s, std::exp(0.01), 1e-8);
    // R should be close to identity
    EXPECT_NEAR(S.R.trace(), 3.0, 1e-8);
}

// ── Umeyama (closed-form Sim3) ────────────────────────────────────────────────

TEST(Sim3Test, UmeyamaIdentity) {
    std::vector<Vec3> pts1 = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const std::vector<Vec3>& pts2 = pts1;  // Same points

    Sim3 S = UmeyamaSim3(pts1, pts2);
    EXPECT_NEAR(S.s, 1.0, 1e-10);
    EXPECT_NEAR(S.R.trace(), 3.0, 1e-10);
    EXPECT_NEAR(S.t.norm(), 0.0, 1e-10);
}

TEST(Sim3Test, UmeyamaScaleOnly) {
    std::vector<Vec3> pts1 = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    std::vector<Vec3> pts2 = {{0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {0, 0, 2}};

    Sim3 S = UmeyamaSim3(pts1, pts2);
    EXPECT_NEAR(S.s, 2.0, 1e-6);
    EXPECT_NEAR(S.R.trace(), 3.0, 1e-6);
    EXPECT_NEAR(S.t.norm(), 0.0, 1e-6);
}

TEST(Sim3Test, UmeyamaRotationAndTranslation) {
    // Create known transform
    Mat3 R = Eigen::AngleAxisd(0.5, Vec3::UnitZ()).toRotationMatrix();
    Vec3 t(1.0, -2.0, 3.0);
    Scalar s = 1.5;

    std::vector<Vec3> pts1;
    pts1.reserve(10);
    for (int i = 0; i < 10; ++i) {
        pts1.push_back(Vec3::Random());
    }

    // Apply the known transform
    std::vector<Vec3> pts2;
    pts2.reserve(pts1.size());
    for (const auto& p : pts1) {
        pts2.push_back(s * (R * p) + t);
    }

    Sim3 S = UmeyamaSim3(pts1, pts2);

    // Check recovery
    EXPECT_NEAR(S.s, s, 1e-6);
    EXPECT_NEAR((S.R - R).norm(), 0.0, 1e-6);
    EXPECT_NEAR((S.t - t).norm(), 0.0, 1e-6);
}

// ── RANSAC Sim3 ───────────────────────────────────────────────────────────────

TEST(Sim3Test, RansacNoOutliers) {
    Mat3 R = Eigen::AngleAxisd(0.3, Vec3(1, 1, 0).normalized()).toRotationMatrix();
    Vec3 t(0.5, -0.3, 0.2);
    Scalar s = 1.2;

    std::vector<Vec3> pts1, pts2;
    for (int i = 0; i < 20; ++i) {
        Vec3 p = Vec3::Random() * 5.0;
        pts1.push_back(p);
        pts2.push_back(s * (R * p) + t);
    }

    Sim3RansacOptions opts;
    opts.min_inliers = 10;
    opts.max_error_3d = 0.01;

    auto result = EstimateSim3Ransac(pts1, pts2, opts);

    EXPECT_TRUE(result.valid);
    EXPECT_GE(result.num_inliers, 15);
    EXPECT_NEAR(result.S.s, s, 1e-4);
    EXPECT_NEAR((result.S.R - R).norm(), 0.0, 1e-4);
    EXPECT_NEAR((result.S.t - t).norm(), 0.0, 1e-4);
}

TEST(Sim3Test, RansacWithOutliers) {
    Mat3 R = Eigen::AngleAxisd(0.3, Vec3::UnitZ()).toRotationMatrix();
    Vec3 t(0.5, -0.3, 0.2);
    Scalar s = 1.2;

    std::vector<Vec3> pts1, pts2;
    for (int i = 0; i < 30; ++i) {
        Vec3 p = Vec3::Random() * 5.0;
        pts1.push_back(p);
        if (i < 20) {
            // Inlier: follows the transform
            pts2.push_back(s * (R * p) + t + 0.001 * Vec3::Random());
        } else {
            // Outlier: random position
            pts2.push_back(Vec3::Random() * 10.0);
        }
    }

    Sim3RansacOptions opts;
    opts.min_inliers = 10;
    opts.max_error_3d = 0.05;

    auto result = EstimateSim3Ransac(pts1, pts2, opts);

    EXPECT_TRUE(result.valid);
    EXPECT_GE(result.num_inliers, 12);
    EXPECT_NEAR(result.S.s, s, 5e-2);
}

TEST(Sim3Test, RansacInsufficientPoints) {
    std::vector<Vec3> pts1 = {{0, 0, 0}, {1, 0, 0}};
    std::vector<Vec3> pts2 = {{0, 0, 0}, {2, 0, 0}};

    auto result = EstimateSim3Ransac(pts1, pts2);
    EXPECT_FALSE(result.valid);
}

TEST(Sim3Test, ComputeFromThreePoints) {
    Mat3 R = Eigen::AngleAxisd(0.2, Vec3::UnitY()).toRotationMatrix();
    Vec3 t(0.1, 0.2, 0.3);
    Scalar s = 1.3;

    Vec3 p1(0, 0, 0), p2(1, 0, 0), p3(0, 1, 0);
    Vec3 q1 = s * (R * p1) + t;
    Vec3 q2 = s * (R * p2) + t;
    Vec3 q3 = s * (R * p3) + t;

    Sim3 S = ComputeSim3FromThree(p1, p2, p3, q1, q2, q3);
    EXPECT_NEAR(S.s, s, 1e-6);
    EXPECT_NEAR((S.R - R).norm(), 0.0, 1e-6);
    EXPECT_NEAR((S.t - t).norm(), 0.0, 1e-6);
}

// ── Sim3 to SE3 conversion ────────────────────────────────────────────────────

TEST(Sim3Test, Sim3ToSE3_DropsScale) {
    Mat3 R = Eigen::AngleAxisd(0.4, Vec3::UnitX()).toRotationMatrix();
    Vec3 t(1, 2, 3);
    Scalar s = 2.5;
    Sim3 S = MakeSim3(R, t, s);

    SE3 T = Sim3ToSE3(S);
    EXPECT_NEAR((T.rotation() - R).norm(), 0.0, 1e-10);
    EXPECT_NEAR((T.translation() - t).norm(), 0.0, 1e-10);
}
