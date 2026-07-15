// =============================================================================
// LocalMapper unit tests
// =============================================================================

#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "slamforge/core/camera.h"
#include "slamforge/core/config.h"
#include "slamforge/core/frame.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"
#include "slamforge/features/orb_extractor.h"
#include "slamforge/mapping/local_mapper.h"

using namespace slamforge;

class LocalMapperTest : public ::testing::Test {
protected:
    void SetUp() override {
        Frame::ResetIdCounter();
        MapPoint::ResetIdCounter();

        Camera::CameraParams params{500, 500, 320, 240, 0, 0, 0, 0, 0, 640, 480};
        camera_ = Camera::FromParams(params);

        image_ = cv::Mat(480, 640, CV_8UC1, cv::Scalar(128));
        cv::rectangle(image_, cv::Rect(100, 100, 80, 80), cv::Scalar(255), -1);
        cv::rectangle(image_, cv::Rect(350, 150, 80, 80), cv::Scalar(0), -1);

        features::OrbExtractor::Options orb_opts;
        orb_opts.num_features = 500;
        extractor_ = std::make_unique<features::OrbExtractor>(orb_opts);

        MappingConfig map_cfg;
        map_cfg.sliding_window_size = 5;
        map_cfg.min_observations = 2;
        map_cfg.max_reprojection_error = 4.0;

        mapper_ = std::make_unique<mapping::LocalMapper>(map_, camera_, map_cfg, *extractor_);
    }

    void TearDown() override {
        if (mapper_ && mapper_->IsRunning()) {
            mapper_->Stop();
        }
    }

    Camera camera_;
    cv::Mat image_;
    std::unique_ptr<features::OrbExtractor> extractor_;
    Map map_;
    std::unique_ptr<mapping::LocalMapper> mapper_;
};

TEST_F(LocalMapperTest, InitialState) {
    EXPECT_FALSE(mapper_->IsRunning());
    EXPECT_FALSE(mapper_->IsFinished());
    EXPECT_EQ(mapper_->QueueSize(), 0);
    EXPECT_TRUE(mapper_->IsAcceptingKeyFrames());
}

TEST_F(LocalMapperTest, StartStop) {
    mapper_->Start();
    EXPECT_TRUE(mapper_->IsRunning());

    mapper_->Stop();
    EXPECT_FALSE(mapper_->IsRunning());
    EXPECT_TRUE(mapper_->IsFinished());
}

TEST_F(LocalMapperTest, RequestStop) {
    mapper_->Start();
    mapper_->RequestStop();

    // Give thread time to finish
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (mapper_->IsRunning()) {
        mapper_->Stop();
    }
    EXPECT_TRUE(mapper_->IsFinished());
}

TEST_F(LocalMapperTest, InsertKeyFrame) {
    auto kf = std::make_shared<KeyFrame>(image_, 0.0, *extractor_, camera_);
    kf->SetPose(SE3::Identity());

    mapper_->InsertKeyFrame(kf);
    EXPECT_EQ(mapper_->QueueSize(), 1);

    // Second insert
    auto kf2 = std::make_shared<KeyFrame>(image_, 0.1, *extractor_, camera_);
    kf2->SetPose(SE3::Identity());
    mapper_->InsertKeyFrame(kf2);
    EXPECT_EQ(mapper_->QueueSize(), 2);
}

TEST_F(LocalMapperTest, SetAcceptKeyFrames) {
    mapper_->SetAcceptKeyFrames(false);
    EXPECT_FALSE(mapper_->IsAcceptingKeyFrames());

    // Insert should be ignored
    auto kf = std::make_shared<KeyFrame>(image_, 0.0, *extractor_, camera_);
    mapper_->InsertKeyFrame(kf);
    EXPECT_EQ(mapper_->QueueSize(), 0);

    mapper_->SetAcceptKeyFrames(true);
    EXPECT_TRUE(mapper_->IsAcceptingKeyFrames());
}

TEST_F(LocalMapperTest, DoubleStartNoCrash) {
    mapper_->Start();
    EXPECT_NO_THROW(mapper_->Start());  // Should be idempotent
    mapper_->Stop();
}

TEST_F(LocalMapperTest, RestartAfterStopIsSafe) {
    mapper_->Start();
    mapper_->Stop();
    ASSERT_TRUE(mapper_->IsFinished());

    EXPECT_NO_THROW(mapper_->Start());
    EXPECT_TRUE(mapper_->IsRunning());
    mapper_->Stop();
    EXPECT_TRUE(mapper_->IsFinished());
}

TEST_F(LocalMapperTest, StopBeforeStart) {
    EXPECT_NO_THROW(mapper_->Stop());  // Should not crash
    EXPECT_FALSE(mapper_->IsRunning());
}
