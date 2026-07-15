// =============================================================================
// Tracker / monocular initializer regression tests
// =============================================================================

#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

#include "litevo/core/camera.h"
#include "litevo/core/frame.h"
#include "litevo/core/keyframe.h"
#include "litevo/core/map_point.h"
#include "litevo/features/orb_extractor.h"
#include "litevo/tracking/initializer.h"
#include "litevo/tracking/matcher.h"
#include "litevo/tracking/tracker.h"

using namespace litevo;

namespace {

constexpr double kReferenceTimestamp = 12.5;
constexpr double kCurrentTimestamp = 12.6;

cv::Mat MakeReferenceImage() {
    cv::Mat image(480, 640, CV_8UC1);
    cv::RNG rng(0x5a17);
    rng.fill(image, cv::RNG::UNIFORM, 0, 256);

    // Add larger stable structures as well as the random texture.  A pure
    // image translation is a deterministic two-view homography with enough
    // parallax for the initializer's two-view checks.
    for (int y = 40; y < image.rows - 40; y += 80) {
        for (int x = 40; x < image.cols - 40; x += 80) {
            cv::rectangle(image, cv::Rect(x, y, 22, 22), cv::Scalar((x + y) % 256), -1);
            cv::circle(image, cv::Point(x + 35, y + 35), 8,
                       cv::Scalar((2 * x + y) % 256), -1);
        }
    }
    return image;
}

cv::Mat TranslateView(const cv::Mat& reference, double x_translation) {
    cv::Mat current;
    const cv::Mat affine =
        (cv::Mat_<double>(2, 3) << 1.0, 0.0, x_translation, 0.0, 1.0, 0.0);
    cv::warpAffine(reference, current, affine, reference.size(), cv::INTER_NEAREST,
                   cv::BORDER_CONSTANT, cv::Scalar(0));
    return current;
}

Camera TestCamera() {
    return Camera(500.0, 500.0, 320.0, 240.0, 640, 480);
}

features::OrbExtractor::Options TestOrbOptions() {
    features::OrbExtractor::Options options;
    options.num_features = 1000;
    options.num_levels = 8;
    return options;
}

TrackingConfig TestTrackingConfig() {
    TrackingConfig config;
    config.min_features_for_tracking = 40;
    config.min_parallax_deg = 0.5;
    config.max_reprojection_error = 3.0;
    config.max_frames_between_kf = 0;  // Promote the third frame in this regression.
    return config;
}

}  // namespace

class TrackingInitializationTest : public ::testing::Test {
protected:
    void SetUp() override {
        Frame::ResetIdCounter();
        MapPoint::ResetIdCounter();
        reference_ = MakeReferenceImage();
        current_ = TranslateView(reference_, -24.0);
    }

    cv::Mat reference_;
    cv::Mat current_;
};

TEST_F(TrackingInitializationTest, InitializerKeepsReferenceAndParallelAssociations) {
    const Camera camera = TestCamera();
    features::OrbExtractor extractor(TestOrbOptions());
    const TrackingConfig tracking_config = TestTrackingConfig();

    tracking::MonocularInitializer::Options options;
    options.min_features = tracking_config.min_features_for_tracking;
    options.min_matches = tracking_config.min_features_for_tracking;
    options.min_parallax_deg = tracking_config.min_parallax_deg;
    options.max_reproj_error = tracking_config.max_reprojection_error;
    tracking::MonocularInitializer initializer(camera, extractor, options);

    Frame reference_frame(reference_, kReferenceTimestamp, extractor, camera);
    Frame current_frame(current_, kCurrentTimestamp, extractor, camera);

    EXPECT_FALSE(initializer.Initialize(reference_frame).success);
    const auto result = initializer.Initialize(current_frame);

    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.model_used, "Homography");
    ASSERT_NE(result.reference_frame, nullptr);
    ASSERT_GT(result.map_points.size(), 0u);
    ASSERT_EQ(result.map_points.size(), result.match_indices_ref.size());
    ASSERT_EQ(result.map_points.size(), result.match_indices_cur.size());
    EXPECT_EQ(result.reference_frame->Id(), reference_frame.Id());
    EXPECT_EQ(result.reference_frame->NumKeyPoints(), reference_frame.NumKeyPoints());

    for (size_t i = 0; i < result.map_points.size(); ++i) {
        const int ref_index = result.match_indices_ref[i];
        const int cur_index = result.match_indices_cur[i];
        ASSERT_GE(ref_index, 0);
        ASSERT_GE(cur_index, 0);
        ASSERT_LT(ref_index, result.reference_frame->NumKeyPoints());
        ASSERT_LT(cur_index, current_frame.NumKeyPoints());
        EXPECT_EQ(tracking::FeatureMatcher::DescriptorDistance(
                      result.map_points[i]->Descriptor(),
                      result.reference_frame->Descriptors().row(ref_index)),
                  0);
    }
}

TEST_F(TrackingInitializationTest, TrackerCreatesUsableTwoViewMap) {
    const Camera camera = TestCamera();
    const TrackingConfig tracking_config = TestTrackingConfig();
    OrbConfig orb_config;
    orb_config.num_features = 1000;

    tracking::Tracker tracker(camera, tracking_config, orb_config);

    EXPECT_FALSE(tracker.Track(reference_, kReferenceTimestamp).has_value());
    const auto pose = tracker.Track(current_, kCurrentTimestamp);

    ASSERT_TRUE(pose.has_value());
    ASSERT_TRUE(tracker.IsInitialized());
    ASSERT_NE(tracker.CurrentFrame(), nullptr);
    EXPECT_NEAR((tracker.CurrentFrame()->Pose().matrix() - pose->matrix()).norm(), 0.0, 1e-12);
    EXPECT_GT(pose->translation().norm(), 1e-6);
    EXPECT_EQ(tracker.NumKeyFrames(), 2);
    EXPECT_GT(tracker.GetMap().MapPointCount(), 0u);
    EXPECT_EQ(static_cast<size_t>(tracker.CurrentFrame()->NumMapPoints()),
              tracker.GetMap().MapPointCount());

    const auto keyframes = tracker.GetMap().GetAllKeyFrames();
    ASSERT_EQ(keyframes.size(), 2u);
    EXPECT_EQ(keyframes[0]->Timestamp(), kReferenceTimestamp);
    EXPECT_EQ(keyframes[1]->Timestamp(), kCurrentTimestamp);
    EXPECT_EQ(keyframes[0]->NumMapPoints(), keyframes[1]->NumMapPoints());

    for (const auto& map_point : tracker.GetMap().GetAllMapPoints()) {
        ASSERT_NE(map_point, nullptr);
        EXPECT_NE(map_point->Id().id, 0u);
        EXPECT_EQ(map_point->Observations(), 2);
        EXPECT_FALSE(map_point->IsBad());
    }

    // Regression for the original lifecycle bug: the freshly triangulated
    // two-view points must be eligible for the very next tracking frame.
    const auto next_pose = tracker.Track(TranslateView(reference_, -48.0), 12.7);
    ASSERT_TRUE(next_pose.has_value());
    EXPECT_FALSE(tracker.IsLost());
    EXPECT_GE(tracker.NumTrackedPoints(), tracking_config.min_features_for_tracking / 3);
    EXPECT_EQ(tracker.NumKeyFrames(), 3);

    const auto keyframes_after_tracking = tracker.GetMap().GetAllKeyFrames();
    ASSERT_EQ(keyframes_after_tracking.size(), 3u);
    const auto& third_keyframe = keyframes_after_tracking.back();
    ASSERT_GT(third_keyframe->NumMapPoints(), 0);
    for (const int feature_index : third_keyframe->GetMapPointIndices()) {
        const auto map_point =
            tracker.GetMap().GetMapPoint(third_keyframe->MapPointIdAt(feature_index));
        ASSERT_NE(map_point, nullptr);
        EXPECT_EQ(map_point->Observations(), 3);
    }
}
