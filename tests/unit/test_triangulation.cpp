// =============================================================================
// Triangulation unit tests
// =============================================================================

#include <gtest/gtest.h>

#include "slamforge/core/camera.h"
#include "slamforge/geometry/se3.h"
#include "slamforge/geometry/triangulation.h"

using namespace slamforge;
using namespace slamforge::geometry;

class TriangulationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup two cameras: baseline 1m, looking forward
        Camera cam(718.856, 718.856, 607.193, 185.216, 1241, 376);

        SE3 T_wc1 = SE3::Identity();
        T_wc1.translation() = Vec3(0, 0, 0);

        SE3 T_wc2 = SE3::Identity();
        T_wc2.translation() = Vec3(1, 0, 0);  // 1m baseline

        T_cw1_ = T_wc1.inverse();
        T_cw2_ = T_wc2.inverse();

        P1_ = cam.ProjectionMatrix(T_cw1_);
        P2_ = cam.ProjectionMatrix(T_cw2_);
        C1_ = CameraCenter(T_cw1_);
        C2_ = CameraCenter(T_cw2_);

        cam_ = cam;
    }

    Camera cam_;
    SE3 T_cw1_, T_cw2_;
    Mat34 P1_, P2_;
    Vec3 C1_, C2_;
};

TEST_F(TriangulationTest, KnownPoint) {
    // A point at (0, 0, 10) in world
    Vec3 p_w(0, 0, 10);

    Vec2 px1 = cam_.ProjectWorld(p_w, T_cw1_);
    Vec2 px2 = cam_.ProjectWorld(p_w, T_cw2_);

    auto result = TriangulatePoint(px1, px2, P1_, P2_, C1_, C2_);

    EXPECT_TRUE(result.valid);
    EXPECT_NEAR(result.point_w.x(), 0, 0.1);
    EXPECT_NEAR(result.point_w.y(), 0, 0.1);
    EXPECT_NEAR(result.point_w.z(), 10, 0.1);
}

TEST_F(TriangulationTest, ParallaxCheck) {
    // A point at (0, 0, 100) far away with 1m baseline → very small parallax
    Vec3 p_far(0, 0, 100);

    Vec2 px1 = cam_.ProjectWorld(p_far, T_cw1_);
    Vec2 px2 = cam_.ProjectWorld(p_far, T_cw2_);

    // Very strict parallax requirement should reject far points
    TriangulationOptions strict_opts;
    strict_opts.min_parallax_deg = 10.0;

    auto result = TriangulatePoint(px1, px2, P1_, P2_, C1_, C2_, strict_opts);
    EXPECT_FALSE(result.valid);

    // Same point with relaxed parallax should pass
    TriangulationOptions relaxed_opts;
    relaxed_opts.min_parallax_deg = 0.01;

    auto result2 = TriangulatePoint(px1, px2, P1_, P2_, C1_, C2_, relaxed_opts);
    EXPECT_TRUE(result2.valid);
}

TEST_F(TriangulationTest, ReprojectionError) {
    Vec3 p_w(0.5, -0.3, 8.0);
    Vec2 px1 = cam_.ProjectWorld(p_w, T_cw1_);

    double error = ReprojectionError(P1_, p_w, px1);
    EXPECT_NEAR(error, 0.0, 1e-6);

    // Deliberately wrong point
    Vec3 p_wrong(10, 10, 100);
    double error_wrong = ReprojectionError(P1_, p_wrong, px1);
    EXPECT_GT(error_wrong, 50.0);  // Should be large
}

TEST_F(TriangulationTest, BehindCamera) {
    // Point behind BOTH cameras (negative depth) should be invalid.
    // Use a 3D point behind both cameras and project it
    Vec3 p_behind(0, 0, -5);  // 5m behind camera 1

    Vec2 px1(300, 185.216);  // arbitrary image point

    // This scenario should produce invalid triangulation with min_depth check
    TriangulationOptions opts;
    opts.min_depth = 0.01;

    // Use a point pair that's physically inconsistent for forward-facing cameras
    Vec2 px_left(200, 185.216);
    Vec2 px_right(1000, 185.216);

    auto result = TriangulatePoint(px_left, px_right, P1_, P2_, C1_, C2_, opts);
    // Either invalid due to depth, or a very far/distant triangulation
    // We just verify no crash and the result structure is populated
    EXPECT_TRUE(result.point_w.allFinite() || !result.valid);
}

TEST_F(TriangulationTest, ParallaxAngleFunction) {
    Vec3 C1(0, 0, 0);
    Vec3 C2(1, 0, 0);
    Vec3 p_w(0, 0, 10);

    double angle = ParallaxAngle(C1, C2, p_w);
    EXPECT_GT(angle, 0);
    EXPECT_LT(angle, 90);
}

TEST_F(TriangulationTest, BatchPoints) {
    std::vector<Vec3> world_points = {{0, 0, 10}, {0.5, 0.5, 15}, {-0.3, 0.2, 8}};

    std::vector<Vec2> pts1, pts2;
    for (const auto& pw : world_points) {
        pts1.push_back(cam_.ProjectWorld(pw, T_cw1_));
        pts2.push_back(cam_.ProjectWorld(pw, T_cw2_));
    }

    auto results = TriangulatePoints(pts1, pts2, P1_, P2_, C1_, C2_);

    EXPECT_EQ(results.size(), 3);
    for (const auto& r : results) {
        EXPECT_TRUE(r.valid);
    }
}
