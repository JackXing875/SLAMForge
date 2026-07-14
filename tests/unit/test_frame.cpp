// =============================================================================
// Frame unit tests
// =============================================================================

#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "litevo/core/frame.h"
#include "litevo/features/orb_extractor.h"

using namespace litevo;

class FrameTest : public ::testing::Test {
protected:
    void SetUp() override {
        Frame::ResetIdCounter();
        // Create a synthetic image with some texture (gradient)
        image_ = cv::Mat(480, 640, CV_8UC1);
        for (int r = 0; r < image_.rows; ++r) {
            for (int c = 0; c < image_.cols; ++c) {
                image_.at<uchar>(r, c) = static_cast<uchar>((r + c) % 256);
            }
        }
        // Add some corners for FAST to detect
        cv::rectangle(image_, cv::Rect(100, 100, 50, 50), cv::Scalar(255), -1);
        cv::rectangle(image_, cv::Rect(300, 200, 50, 50), cv::Scalar(0), -1);
        cv::rectangle(image_, cv::Rect(200, 350, 50, 50), cv::Scalar(128), -1);

        camera_ = Camera(500, 500, 320, 240, 640, 480);
        orb_ = features::OrbExtractor();
    }

    cv::Mat image_;
    Camera camera_;
    features::OrbExtractor orb_;
};

TEST_F(FrameTest, Construction) {
    Frame frame(image_, 0.0, orb_, camera_);
    EXPECT_GT(frame.NumKeyPoints(), 0);
    EXPECT_FALSE(frame.Descriptors().empty());
    EXPECT_EQ(frame.Timestamp(), 0.0);
    EXPECT_FALSE(frame.IsKeyFrame());
    // Descriptors should have one row per keypoint, 32 bytes each
    EXPECT_EQ(frame.Descriptors().rows, frame.NumKeyPoints());
    EXPECT_EQ(frame.Descriptors().cols, 32);
}

TEST_F(FrameTest, UniqueIds) {
    Frame f1(image_, 0.0, orb_, camera_);
    Frame f2(image_, 0.1, orb_, camera_);
    EXPECT_NE(f1.Id(), f2.Id());
    EXPECT_EQ(f1.Id().id, 0);
    EXPECT_EQ(f2.Id().id, 1);
}

TEST_F(FrameTest, PoseDefaults) {
    Frame frame(image_, 0.0, orb_, camera_);
    // Default pose should be identity
    EXPECT_NEAR(frame.Pose().translation().norm(), 0.0, 1e-10);
}

TEST_F(FrameTest, SetPose) {
    Frame frame(image_, 0.0, orb_, camera_);
    SE3 pose = SE3::Identity();
    pose.translation() = Vec3(1, 2, 3);
    frame.SetPose(pose);
    EXPECT_NEAR(frame.Pose().translation().x(), 1.0, 1e-10);
    EXPECT_NEAR(frame.Pose().translation().y(), 2.0, 1e-10);
    EXPECT_NEAR(frame.Pose().translation().z(), 3.0, 1e-10);
}

TEST_F(FrameTest, CameraCenter) {
    Frame frame(image_, 0.0, orb_, camera_);
    SE3 pose = SE3::Identity();
    pose.translation() = Vec3(0, 0, 5);
    frame.SetPose(pose);
    Vec3 center = frame.CameraCenter();
    // For Tcw = [I | (0,0,5)], center = -I^T * (0,0,5) = (0,0,-5)
    EXPECT_NEAR(center.z(), -5.0, 1e-10);
}

TEST_F(FrameTest, MapPointAssociations) {
    Frame frame(image_, 0.0, orb_, camera_);
    EXPECT_EQ(frame.NumMapPoints(), 0);

    // Set some map point associations
    if (frame.NumKeyPoints() > 0) {
        frame.SetMapPointId(0, MapPointId{42});
        EXPECT_EQ(frame.MapPointIdAt(0).id, 42);
        EXPECT_EQ(frame.NumMapPoints(), 1);

        // Out of bounds should return null
        EXPECT_EQ(frame.MapPointIdAt(-1).id, 0);
        EXPECT_EQ(frame.MapPointIdAt(frame.NumKeyPoints() + 10).id, 0);
    }
}

TEST_F(FrameTest, KeyframeFlag) {
    Frame frame(image_, 0.0, orb_, camera_);
    EXPECT_FALSE(frame.IsKeyFrame());
    frame.SetKeyFrame(true);
    EXPECT_TRUE(frame.IsKeyFrame());
}

TEST_F(FrameTest, FeatureGridAccess) {
    Frame frame(image_, 0.0, orb_, camera_);

    // Get features in a region — should not crash
    auto feats = frame.GetFeaturesInArea(320.0f, 240.0f, 100.0f);
    // Just verify it returns something reasonable
    EXPECT_NO_THROW(frame.GetFeaturesInArea(0.0f, 0.0f, 50.0f));
    EXPECT_NO_THROW(frame.GetFeaturesInArea(640.0f, 480.0f, 50.0f));
}

TEST_F(FrameTest, UndistortedKeypoints) {
    // Camera without distortion — undistorted should match original
    Frame frame(image_, 0.0, orb_, camera_);
    EXPECT_EQ(frame.KeyPoints().size(), frame.KeyPointsUndistorted().size());
    // For a distortion-free camera, positions should be identical
    for (size_t i = 0; i < frame.KeyPoints().size(); ++i) {
        EXPECT_FLOAT_EQ(frame.KeyPoints()[i].pt.x,
                        frame.KeyPointsUndistorted()[i].pt.x);
        EXPECT_FLOAT_EQ(frame.KeyPoints()[i].pt.y,
                        frame.KeyPointsUndistorted()[i].pt.y);
    }
}

TEST_F(FrameTest, DistortedCamera) {
    Camera cam_dist(458.654, 457.296, 367.215, 248.375, 752, 480);
    cam_dist.SetDistortion(-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);

    cv::Mat img(480, 752, CV_8UC1);
    for (int r = 0; r < img.rows; ++r)
        for (int c = 0; c < img.cols; ++c)
            img.at<uchar>(r, c) = static_cast<uchar>((r + c) % 256);
    cv::rectangle(img, cv::Rect(100, 100, 50, 50), cv::Scalar(255), -1);
    cv::rectangle(img, cv::Rect(300, 200, 50, 50), cv::Scalar(0), -1);
    cv::rectangle(img, cv::Rect(500, 300, 50, 50), cv::Scalar(128), -1);

    Frame frame(img, 0.0, orb_, cam_dist);
    EXPECT_EQ(frame.KeyPoints().size(), frame.KeyPointsUndistorted().size());

    // With distortion, positions should differ for at least some points
    bool any_different = false;
    for (size_t i = 0; i < frame.KeyPoints().size(); ++i) {
        float dx = frame.KeyPoints()[i].pt.x - frame.KeyPointsUndistorted()[i].pt.x;
        float dy = frame.KeyPoints()[i].pt.y - frame.KeyPointsUndistorted()[i].pt.y;
        if (std::abs(dx) > 0.1f || std::abs(dy) > 0.1f) {
            any_different = true;
            break;
        }
    }
    EXPECT_TRUE(any_different);
}
