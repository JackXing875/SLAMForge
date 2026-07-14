// =============================================================================
// Unit tests for LiteVO core types
// =============================================================================

#include <gtest/gtest.h>

#include "litevo/core/types.h"

// ── Vector types ─────────────────────────────────────────────────────────────

TEST(Vec3Test, DefaultConstructor) {
    litevo::Vec3 v;
    EXPECT_EQ(v.size(), 3);
}

TEST(Vec3Test, InitializedValues) {
    litevo::Vec3 v(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(v.x(), 1.0);
    EXPECT_DOUBLE_EQ(v.y(), 2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.0);
}

TEST(Vec3Test, ArithmeticOps) {
    litevo::Vec3 a(1.0, 2.0, 3.0);
    litevo::Vec3 b(4.0, 5.0, 6.0);

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
    litevo::Vec3 v(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(v.norm(), 5.0);
}

// ── Pose ─────────────────────────────────────────────────────────────────────

TEST(PoseTest, DefaultPose) {
    litevo::Pose p;
    EXPECT_DOUBLE_EQ(p.position.x(), 0.0);
    EXPECT_DOUBLE_EQ(p.position.y(), 0.0);
    EXPECT_DOUBLE_EQ(p.position.z(), 0.0);
    EXPECT_DOUBLE_EQ(p.orientation.w(), 1.0);  // Identity quaternion
}

TEST(PoseTest, ToFromSE3Roundtrip) {
    litevo::Pose p;
    p.position = litevo::Vec3(1.0, 2.0, 3.0);
    p.orientation = litevo::Quaternion::Identity();

    auto T = p.ToSE3();
    auto p2 = litevo::Pose::FromSE3(T);

    EXPECT_NEAR(p2.position.x(), p.position.x(), 1e-12);
    EXPECT_NEAR(p2.position.y(), p.position.y(), 1e-12);
    EXPECT_NEAR(p2.position.z(), p.position.z(), 1e-12);
}

// ── FrameId / MapPointId ─────────────────────────────────────────────────────

TEST(FrameIdTest, Equality) {
    litevo::FrameId a{42};
    litevo::FrameId b{42};
    litevo::FrameId c{43};

    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
}

TEST(MapPointIdTest, Hashable) {
    std::unordered_map<litevo::MapPointId, std::string, litevo::MapPointIdHash> m;
    m[litevo::MapPointId{1}] = "point_1";
    m[litevo::MapPointId{2}] = "point_2";

    EXPECT_EQ(m.at(litevo::MapPointId{1}), "point_1");
    EXPECT_EQ(m.at(litevo::MapPointId{2}), "point_2");
}

// ── SE3 ──────────────────────────────────────────────────────────────────────

TEST(SE3Test, Identity) {
    litevo::SE3 T = litevo::SE3::Identity();
    litevo::Vec3 p(1.0, 0.0, 0.0);

    auto q = T * p;
    EXPECT_NEAR(q.x(), 1.0, 1e-12);
    EXPECT_NEAR(q.y(), 0.0, 1e-12);
    EXPECT_NEAR(q.z(), 0.0, 1e-12);
}

TEST(SE3Test, Translation) {
    litevo::SE3 T = litevo::SE3::Identity();
    T.translation() = litevo::Vec3(10.0, 0.0, 0.0);

    litevo::Vec3 p(0.0, 0.0, 0.0);
    auto q = T * p;

    EXPECT_NEAR(q.x(), 10.0, 1e-12);
    EXPECT_NEAR(q.y(), 0.0, 1e-12);
    EXPECT_NEAR(q.z(), 0.0, 1e-12);
}

TEST(SE3Test, Compose) {
    litevo::SE3 T1 = litevo::SE3::Identity();
    T1.translation() = litevo::Vec3(1.0, 0.0, 0.0);

    litevo::SE3 T2 = litevo::SE3::Identity();
    T2.translation() = litevo::Vec3(0.0, 2.0, 0.0);

    auto T12 = T1 * T2;
    litevo::Vec3 p(0.0, 0.0, 0.0);
    auto q = T12 * p;

    EXPECT_NEAR(q.x(), 1.0, 1e-12);
    EXPECT_NEAR(q.y(), 2.0, 1e-12);
    EXPECT_NEAR(q.z(), 0.0, 1e-12);
}

TEST(SE3Test, Inverse) {
    litevo::SE3 T = litevo::SE3::Identity();
    T.translation() = litevo::Vec3(5.0, 0.0, 0.0);

    auto T_inv = T.inverse();
    litevo::Vec3 p(0.0, 0.0, 0.0);
    auto q = T_inv * p;

    EXPECT_NEAR(q.x(), -5.0, 1e-12);
}
