// =============================================================================
// SE(3) Lie algebra unit tests
// =============================================================================

#include <gtest/gtest.h>

#include "slamforge/geometry/se3.h"

using namespace slamforge;
using namespace slamforge::geometry;

TEST(SE3Test, MakeSE3_Rt) {
    Mat3 R = Mat3::Identity();
    Vec3 t(1, 2, 3);
    SE3 T = MakeSE3(R, t);
    EXPECT_NEAR(T.translation().x(), 1, 1e-12);
    EXPECT_NEAR(T.translation().y(), 2, 1e-12);
    EXPECT_NEAR(T.translation().z(), 3, 1e-12);
}

TEST(SE3Test, CameraCenter) {
    // Camera at (1,0,0), looking at origin
    SE3 T_cw = SE3::Identity();
    T_cw.translation() = Vec3(0, 0, 5);  // Camera at z=5 in world

    Vec3 center = CameraCenter(T_cw);
    // For T_cw = [I | (0,0,5)], center = -I^T*(0,0,5) = (0,0,-5)
    EXPECT_NEAR(center.z(), -5.0, 1e-12);
}

TEST(SE3Test, ExpLogRoundTrip) {
    // Test Exp(Log(T)) = T for various transformations
    for (int i = 0; i < 10; ++i) {
        SE3 T = SE3::Identity();
        T.translation() = Vec3(i * 0.5, -i * 0.2, 2.0 + i * 0.1);
        T.linear() = Eigen::AngleAxisd(i * 0.3, Vec3(0, 0, 1).normalized()).toRotationMatrix();

        Vec6 twist = LogSE3(T);
        SE3 T2 = ExpSE3(twist);

        // Check that Exp(Log(T)) ≈ T
        EXPECT_NEAR((T2.translation() - T.translation()).norm(), 0, 5e-2);
        EXPECT_NEAR((T2.rotation() - T.rotation()).norm(), 0, 1e-1);
    }
}

TEST(SE3Test, LogExpRoundTrip) {
    // Test Log(Exp(twist)) = twist for small twists
    Vec6 twist;
    twist << 0.1, 0.2, 0.05, -0.01, 0.02, 0.005;
    SE3 T = ExpSE3(twist);
    Vec6 twist2 = LogSE3(T);

    EXPECT_NEAR((twist - twist2).norm(), 0, 1e-1);
}

TEST(SE3Test, IdentityExpLog) {
    SE3 I = SE3::Identity();
    Vec6 zero = LogSE3(I);
    EXPECT_NEAR(zero.norm(), 0, 1e-10);

    SE3 I2 = ExpSE3(zero);
    EXPECT_NEAR((I2.rotation() - Mat3::Identity()).norm(), 0, 1e-10);
    EXPECT_NEAR(I2.translation().norm(), 0, 1e-10);
}

TEST(SE3Test, SkewSymmetric_Vee) {
    Vec3 v(1, 2, 3);
    Mat3 v_hat = SkewSymmetric(v);
    Vec3 v2 = VeeOperator(v_hat);
    EXPECT_NEAR(v2.x(), v.x(), 1e-12);
    EXPECT_NEAR(v2.y(), v.y(), 1e-12);
    EXPECT_NEAR(v2.z(), v.z(), 1e-12);
}

TEST(SE3Test, RelativePose) {
    SE3 T1 = SE3::Identity();
    T1.translation() = Vec3(1, 0, 0);

    SE3 T2 = SE3::Identity();
    T2.translation() = Vec3(3, 0, 0);

    SE3 T_rel = RelativePose(T1, T2);
    EXPECT_NEAR(T_rel.translation().x(), 2.0, 1e-10);
}

TEST(SE3Test, Adjoint) {
    SE3 T = SE3::Identity();
    T.translation() = Vec3(1, 2, 3);

    Mat6 Adj = AdjointSE3(T);
    // The adjoint of a pure translation should have R=I in the blocks
    EXPECT_NEAR(Adj(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(Adj(1, 1), 1.0, 1e-12);
    EXPECT_NEAR(Adj(2, 2), 1.0, 1e-12);
}

TEST(SE3Test, TransformPoints) {
    SE3 T = SE3::Identity();
    T.translation() = Vec3(0, 0, 10);

    std::vector<Vec3> pts_w = {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 0)};
    std::vector<Vec3> pts_c;
    TransformPoints(T, pts_w, pts_c);

    EXPECT_EQ(pts_c.size(), pts_w.size());
    EXPECT_NEAR(pts_c[2].z(), 10.0, 1e-10);
}
