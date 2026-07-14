// =============================================================================
// Bundle Adjustment unit tests
// =============================================================================

#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

#include "litevo/core/camera.h"
#include "litevo/core/frame.h"
#include "litevo/core/keyframe.h"
#include "litevo/core/map_point.h"
#include "litevo/features/orb_extractor.h"
#include "litevo/optimization/ba.h"

using namespace litevo;

class BundleAdjustmentTest : public ::testing::Test {
protected:
    void SetUp() override {
        Frame::ResetIdCounter();
        MapPoint::ResetIdCounter();

        // Camera with known intrinsics
        Camera::CameraParams params{500, 500, 320, 240, 0, 0, 0, 0, 0, 640, 480};
        camera_ = Camera::FromParams(params);

        // Create synthetic image
        image_ = cv::Mat(480, 640, CV_8UC1, cv::Scalar(128));
        cv::rectangle(image_, cv::Rect(100, 100, 80, 80), cv::Scalar(255), -1);
        cv::rectangle(image_, cv::Rect(350, 150, 80, 80), cv::Scalar(0), -1);

        features::OrbExtractor::Options orb_opts;
        orb_opts.num_features = 500;
        extractor_ = std::make_unique<features::OrbExtractor>(orb_opts);
    }

    Camera camera_;
    cv::Mat image_;
    std::unique_ptr<features::OrbExtractor> extractor_;
};

TEST_F(BundleAdjustmentTest, EmptyInput) {
    optimization::LocalBundleAdjuster adjuster(camera_);
    std::vector<KeyFrame*> local_kfs;
    std::vector<KeyFrame*> fixed_kfs;
    std::vector<MapPoint*> mps;
    int removed = adjuster.Optimize(local_kfs, fixed_kfs, mps);
    EXPECT_EQ(removed, 0);
}

TEST_F(BundleAdjustmentTest, SingleKfNoOp) {
    // BA with one KF and no map points should be a no-op
    KeyFrame kf(image_, 0.0, *extractor_, camera_);
    kf.SetPose(SE3::Identity());

    optimization::LocalBundleAdjuster adjuster(camera_);
    std::vector<KeyFrame*> local_kfs = {&kf};
    std::vector<KeyFrame*> fixed_kfs;
    std::vector<MapPoint*> mps;

    int removed = adjuster.Optimize(local_kfs, fixed_kfs, mps);
    EXPECT_EQ(removed, 0);
}

TEST_F(BundleAdjustmentTest, EmptyMapPoints) {
    KeyFrame kf1(image_, 0.0, *extractor_, camera_);
    kf1.SetPose(SE3::Identity());

    auto img2 = image_.clone();
    cv::rectangle(img2, cv::Rect(150, 150, 80, 80), cv::Scalar(128), -1);
    KeyFrame kf2(img2, 1.0, *extractor_, camera_);
    kf2.SetPose(SE3::Identity());

    optimization::LocalBundleAdjuster adjuster(camera_);
    std::vector<KeyFrame*> local_kfs = {&kf1, &kf2};
    std::vector<KeyFrame*> fixed_kfs;
    std::vector<MapPoint*> mps;

    // No residuals to optimize (no map points), should not crash
    int removed = adjuster.Optimize(local_kfs, fixed_kfs, mps);
    EXPECT_EQ(removed, 0);
}
