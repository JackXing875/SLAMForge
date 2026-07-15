// =============================================================================
// Unit tests for SLAMForge core types
// =============================================================================

#include <gtest/gtest.h>

#include "slamforge/core/types.h"

// ── Vector types ─────────────────────────────────────────────────────────────

TEST(Vec3Test, DefaultConstructor) {
    slamforge::Vec3 v;
    EXPECT_EQ(v.size(), 3);
}

TEST(Vec3Test, InitializedValues) {
    slamforge::Vec3 v(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(v.x(), 1.0);
    EXPECT_DOUBLE_EQ(v.y(), 2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.0);
}

TEST(Vec3Test, ArithmeticOps) {
    slamforge::Vec3 a(1.0, 2.0, 3.0);
    slamforge::Vec3 b(4.0, 5.0, 6.0);

    auto sum = a + b;
    EXPECT_DOUBLE_EQ(sum.x(), 5.0);
    EXPECT_DOUBLE_EQ(sum.y(), 7.0);
    EXPECT_DOUBLE_EQ(sum.z(), 9.0);

    auto diff = b - a;
    EXPECT_DOUBLE_EQ(diff.x(), 3.0);
    EXPECT_DOUBLE_EQ(diff.y(), 3.0);
    EXPECT_DOUBLE_EQ(diff.z(), 3.0);
}

TEST(Vec3Test, Norm) {
    slamforge::Vec3 v(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(v.norm(), 5.0);
}

// ── Pose ─────────────────────────────────────────────────────────────────────

TEST(PoseTest, DefaultPose) {
    slamforge::Pose p;
    EXPECT_DOUBLE_EQ(p.position.x(), 0.0);
    EXPECT_DOUBLE_EQ(p.position.y(), 0.0);
    EXPECT_DOUBLE_EQ(p.position.z(), 0.0);
    EXPECT_DOUBLE_EQ(p.orientation.w(), 1.0);  // Identity quaternion
}

TEST(PoseTest, ToFromSE3Roundtrip) {
    slamforge::Pose p;
    p.position = slamforge::Vec3(1.0, 2.0, 3.0);
    p.orientation = slamforge::Quaternion::Identity();

    auto T = p.ToSE3();
    auto p2 = slamforge::Pose::FromSE3(T);

    EXPECT_NEAR(p2.position.x(), p.position.x(), 1e-12);
    EXPECT_NEAR(p2.position.y(), p.position.y(), 1e-12);
    EXPECT_NEAR(p2.position.z(), p.position.z(), 1e-12);
}

// ── FrameId / MapPointId ─────────────────────────────────────────────────────

TEST(FrameIdTest, Equality) {
    slamforge::FrameId a{42};
    slamforge::FrameId b{42};
    slamforge::FrameId c{43};

    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
}

TEST(MapPointIdTest, Hashable) {
    std::unordered_map<slamforge::MapPointId, std::string, slamforge::MapPointIdHash> m;
    m[slamforge::MapPointId{1}] = "point_1";
    m[slamforge::MapPointId{2}] = "point_2";

    EXPECT_EQ(m.at(slamforge::MapPointId{1}), "point_1");
    EXPECT_EQ(m.at(slamforge::MapPointId{2}), "point_2");
}

// ── SE3 ──────────────────────────────────────────────────────────────────────

TEST(SE3Test, Identity) {
    slamforge::SE3 T = slamforge::SE3::Identity();
    slamforge::Vec3 p(1.0, 0.0, 0.0);

    auto q = T * p;
    EXPECT_NEAR(q.x(), 1.0, 1e-12);
    EXPECT_NEAR(q.y(), 0.0, 1e-12);
    EXPECT_NEAR(q.z(), 0.0, 1e-12);
}

TEST(SE3Test, Translation) {
    slamforge::SE3 T = slamforge::SE3::Identity();
    T.translation() = slamforge::Vec3(10.0, 0.0, 0.0);

    slamforge::Vec3 p(0.0, 0.0, 0.0);
    auto q = T * p;

    EXPECT_NEAR(q.x(), 10.0, 1e-12);
    EXPECT_NEAR(q.y(), 0.0, 1e-12);
    EXPECT_NEAR(q.z(), 0.0, 1e-12);
}

TEST(SE3Test, Compose) {
    slamforge::SE3 T1 = slamforge::SE3::Identity();
    T1.translation() = slamforge::Vec3(1.0, 0.0, 0.0);

    slamforge::SE3 T2 = slamforge::SE3::Identity();
    T2.translation() = slamforge::Vec3(0.0, 2.0, 0.0);

    auto T12 = T1 * T2;
    slamforge::Vec3 p(0.0, 0.0, 0.0);
    auto q = T12 * p;

    EXPECT_NEAR(q.x(), 1.0, 1e-12);
    EXPECT_NEAR(q.y(), 2.0, 1e-12);
    EXPECT_NEAR(q.z(), 0.0, 1e-12);
}

TEST(SE3Test, Inverse) {
    slamforge::SE3 T = slamforge::SE3::Identity();
    T.translation() = slamforge::Vec3(5.0, 0.0, 0.0);

    auto T_inv = T.inverse();
    slamforge::Vec3 p(0.0, 0.0, 0.0);
    auto q = T_inv * p;

    EXPECT_NEAR(q.x(), -5.0, 1e-12);
}
