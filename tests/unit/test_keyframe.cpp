// =============================================================================
// KeyFrame unit tests
// =============================================================================

#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

#include "litevo/core/camera.h"
#include "litevo/core/config.h"
#include "litevo/core/frame.h"
#include "litevo/core/keyframe.h"
#include "litevo/core/map_point.h"
#include "litevo/features/orb_extractor.h"

using namespace litevo;

class KeyFrameTest : public ::testing::Test {
protected:
    void SetUp() override {
        Frame::ResetIdCounter();
        MapPoint::ResetIdCounter();

        // Create a synthetic image (small gradient for FAST to detect)
        image_ = cv::Mat(480, 640, CV_8UC1, cv::Scalar(128));
        cv::rectangle(image_, cv::Rect(100, 100, 50, 50), cv::Scalar(255), -1);
        cv::rectangle(image_, cv::Rect(300, 200, 50, 50), cv::Scalar(0), -1);

        features::OrbExtractor::Options orb_opts;
        orb_opts.num_features = 500;
        extractor_ = std::make_unique<features::OrbExtractor>(orb_opts);

        Camera::CameraParams params{500, 500, 320, 240, 0, 0, 0, 0, 0, 640, 480};
        camera_ = Camera::FromParams(params);

        frame_ = std::make_shared<Frame>(image_, 0.0, *extractor_, camera_);
        frame_->SetPose(SE3::Identity());
    }

    cv::Mat image_;
    std::unique_ptr<features::OrbExtractor> extractor_;
    Camera camera_;
    std::shared_ptr<Frame> frame_;
};

TEST_F(KeyFrameTest, ConstructionFromFrame) {
    KeyFrame kf(*frame_);
    EXPECT_EQ(kf.Id(), frame_->Id());
    EXPECT_TRUE(kf.IsKeyFrame());
    EXPECT_FALSE(kf.IsBad());
    EXPECT_EQ(kf.NumKeyPoints(), frame_->NumKeyPoints());
}

TEST_F(KeyFrameTest, ConstructionDirect) {
    KeyFrame kf(image_, 1.0, *extractor_, camera_);
    EXPECT_TRUE(kf.IsKeyFrame());
    EXPECT_GT(kf.NumKeyPoints(), 0);
}

TEST_F(KeyFrameTest, CovisibilityAddAndGet) {
    KeyFrame kf1(*frame_);
    KeyFrame kf2(image_, 1.0, *extractor_, camera_);

    kf1.AddConnection(&kf2, 50);
    EXPECT_EQ(kf1.GetWeight(&kf2), 50);
}

TEST_F(KeyFrameTest, BestCovisibilityKeyFrames) {
    KeyFrame kf1(*frame_);
    KeyFrame kf2(image_, 1.0, *extractor_, camera_);

    // Create more keyframes for sorting
    auto img2 = image_.clone();
    cv::rectangle(img2, cv::Rect(200, 150, 50, 50), cv::Scalar(255), -1);
    KeyFrame kf3(img2, 2.0, *extractor_, camera_);

    kf1.AddConnection(&kf2, 100);
    kf1.AddConnection(&kf3, 50);

    auto best = kf1.GetBestCovisibilityKeyFrames(2);
    ASSERT_EQ(best.size(), 2u);
    EXPECT_EQ(best[0], &kf2);  // Higher weight first
    EXPECT_EQ(best[1], &kf3);
}

TEST_F(KeyFrameTest, CovisiblesByWeight) {
    KeyFrame kf1(*frame_);
    KeyFrame kf2(image_, 1.0, *extractor_, camera_);
    KeyFrame kf3(image_, 2.0, *extractor_, camera_);

    kf1.AddConnection(&kf2, 100);
    kf1.AddConnection(&kf3, 10);

    auto c1 = kf1.GetCovisiblesByWeight(50);
    EXPECT_EQ(c1.size(), 1u);
    EXPECT_EQ(c1[0], &kf2);

    auto c2 = kf1.GetCovisiblesByWeight(5);
    EXPECT_EQ(c2.size(), 2u);
}

TEST_F(KeyFrameTest, EraseConnection) {
    KeyFrame kf1(*frame_);
    KeyFrame kf2(image_, 1.0, *extractor_, camera_);
    kf1.AddConnection(&kf2, 50);
    kf1.EraseConnection(&kf2);
    EXPECT_EQ(kf1.GetWeight(&kf2), 0);
}

TEST_F(KeyFrameTest, SpanningTree) {
    KeyFrame kf1(*frame_);
    KeyFrame kf2(image_, 1.0, *extractor_, camera_);

    kf2.SetParent(&kf1);
    EXPECT_EQ(kf2.GetParent(), &kf1);

    kf1.AddChild(&kf2);
    EXPECT_TRUE(kf1.HasChild(&kf2));
    EXPECT_EQ(kf1.GetChildren().size(), 1u);

    kf1.EraseChild(&kf2);
    EXPECT_FALSE(kf1.HasChild(&kf2));
}

TEST_F(KeyFrameTest, BadFlag) {
    KeyFrame kf(*frame_);
    EXPECT_FALSE(kf.IsBad());
    kf.SetBad(true);
    EXPECT_TRUE(kf.IsBad());
}
