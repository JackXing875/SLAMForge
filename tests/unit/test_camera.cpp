// =============================================================================
// Camera model unit tests
// =============================================================================

#include <gtest/gtest.h>

#include "litevo/core/camera.h"

using namespace litevo;

class CameraTest : public ::testing::Test {
protected:
    void SetUp() override {
        // KITTI-like camera
        cam_ = Camera(718.856, 718.856, 607.193, 185.216, 1241, 376);
    }
    Camera cam_;
};

TEST_F(CameraTest, IntrinsicsAccessors) {
    EXPECT_DOUBLE_EQ(cam_.fx(), 718.856);
    EXPECT_DOUBLE_EQ(cam_.fy(), 718.856);
    EXPECT_DOUBLE_EQ(cam_.cx(), 607.193);
    EXPECT_DOUBLE_EQ(cam_.cy(), 185.216);
    EXPECT_EQ(cam_.width(), 1241);
    EXPECT_EQ(cam_.height(), 376);
}

TEST_F(CameraTest, ProjectAtPrincipalPoint) {
    // A point on the optical axis should project to the principal point
    Vec3 p_cam(0, 0, 10.0);
    Vec2 pixel = cam_.Project(p_cam);
    EXPECT_NEAR(pixel.x(), cam_.cx(), 1e-6);
    EXPECT_NEAR(pixel.y(), cam_.cy(), 1e-6);
}

TEST_F(CameraTest, ProjectBehindCamera) {
    // Points behind the camera should return (-1, -1)
    Vec3 p_cam(1, 1, -1.0);
    Vec2 pixel = cam_.Project(p_cam);
    EXPECT_DOUBLE_EQ(pixel.x(), -1.0);
    EXPECT_DOUBLE_EQ(pixel.y(), -1.0);
}

TEST_F(CameraTest, ProjectRoundTrip) {
    // Unproject → project should be identity for the same point
    Vec2 pixel(500, 200);
    Vec3 ray = cam_.Unproject(pixel);
    // Place a point along the ray at known depth
    Vec3 p_cam = ray * 20.0;
    Vec2 reproj = cam_.Project(p_cam);
    EXPECT_NEAR(reproj.x(), pixel.x(), 1e-6);
    EXPECT_NEAR(reproj.y(), pixel.y(), 1e-6);
}

TEST_F(CameraTest, UnprojectUnitNorm) {
    Vec2 pixel(300, 150);
    Vec3 ray = cam_.Unproject(pixel);
    EXPECT_NEAR(ray.norm(), 1.0, 1e-9);
}

TEST_F(CameraTest, IsInImage) {
    EXPECT_TRUE(cam_.IsInImage(Vec2(10, 10)));
    EXPECT_TRUE(cam_.IsInImage(Vec2(1240, 375)));
    EXPECT_FALSE(cam_.IsInImage(Vec2(-1, 100)));
    EXPECT_FALSE(cam_.IsInImage(Vec2(100, -1)));
    EXPECT_FALSE(cam_.IsInImage(Vec2(1242, 100)));
    EXPECT_FALSE(cam_.IsInImage(Vec2(100, 377)));
}

TEST_F(CameraTest, IsInImageBorder) {
    EXPECT_FALSE(cam_.IsInImage(Vec2(5, 10), 10));
    EXPECT_TRUE(cam_.IsInImage(Vec2(10, 10), 10));
}

TEST_F(CameraTest, ProjectWorld) {
    SE3 T_cw = SE3::Identity();
    T_cw.translation() = Vec3(0, 0, 0);

    Vec3 p_w(0.5, 0.3, 10.0);
    Vec2 pixel = cam_.ProjectWorld(p_w, T_cw);
    EXPECT_NEAR(pixel.x(), cam_.cx() + cam_.fx() * 0.5 / 10.0, 1e-4);
    EXPECT_NEAR(pixel.y(), cam_.cy() + cam_.fy() * 0.3 / 10.0, 1e-4);
}

TEST_F(CameraTest, ProjectionMatrix) {
    SE3 T_cw = SE3::Identity();
    T_cw.translation() = Vec3(1, 2, 3);

    Mat34 P = cam_.ProjectionMatrix(T_cw);
    EXPECT_EQ(P.rows(), 3);
    EXPECT_EQ(P.cols(), 4);
    EXPECT_NEAR(P(0, 3), cam_.fx() * 1 + cam_.cx() * 3, 1e-6);
}

TEST_F(CameraTest, DistortionRoundTrip) {
    Camera cam_dist(458.654, 457.296, 367.215, 248.375, 752, 480);
    cam_dist.SetDistortion(-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0);

    // Test a few points
    std::vector<Vec2> test_pixels = {{400, 250}, {100, 100}, {700, 400}, {367.215, 248.375}};
    for (const auto& px : test_pixels) {
        Vec2 norm((px.x() - cam_dist.cx()) / cam_dist.fx(),
                  (px.y() - cam_dist.cy()) / cam_dist.fy());
        Vec2 distorted = cam_dist.Distort(norm);
        Vec2 undistorted = cam_dist.Undistort(distorted);
        EXPECT_NEAR(undistorted.x(), norm.x(), 1e-4);
        EXPECT_NEAR(undistorted.y(), norm.y(), 1e-4);
    }
}

TEST_F(CameraTest, NoDistortionIdentity) {
    Vec2 norm(0.1, -0.05);
    Vec2 distorted = cam_.Distort(norm);
    EXPECT_NEAR(distorted.x(), norm.x(), 1e-12);
    EXPECT_NEAR(distorted.y(), norm.y(), 1e-12);
}

TEST_F(CameraTest, UndistortedProjectionMatchesFeatureCoordinateSystem) {
    Camera cam_dist(458.654, 457.296, 367.215, 248.375, 752, 480);
    cam_dist.SetDistortion(-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0);

    const Vec3 point_cam(1.0, 0.5, 4.0);
    const Vec2 undistorted = cam_dist.ProjectUndistorted(point_cam);
    const Vec2 distorted = cam_dist.Project(point_cam);

    EXPECT_NEAR(undistorted.x(), cam_dist.cx() + cam_dist.fx() * 0.25, 1e-9);
    EXPECT_NEAR(undistorted.y(), cam_dist.cy() + cam_dist.fy() * 0.125, 1e-9);
    EXPECT_GT((undistorted - distorted).norm(), 1e-3);

    SE3 T_cw = SE3::Identity();
    EXPECT_NEAR((cam_dist.ProjectWorldUndistorted(point_cam, T_cw) - undistorted).norm(), 0.0,
                1e-12);
}
