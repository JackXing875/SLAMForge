// =============================================================================
// Loop-closing worker lifecycle tests
// =============================================================================

#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

#include <memory>

#include "slamforge/core/camera.h"
#include "slamforge/core/config.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"
#include "slamforge/features/orb_extractor.h"
#include "slamforge/loop_closing/corrector.h"
#include "slamforge/loop_closing/global_ba.h"
#include "slamforge/loop_closing/loop_closing.h"
#include "slamforge/mapping/local_mapper.h"

namespace slamforge {
namespace {

Camera MakeCamera() {
    Camera::CameraParams params{500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0, 640, 480};
    return Camera::FromParams(params);
}

std::shared_ptr<KeyFrame> MakeKeyFrame(const Camera& camera, features::OrbExtractor& extractor) {
    cv::Mat image(480, 640, CV_8UC1, cv::Scalar(128));
    cv::rectangle(image, cv::Rect(80, 80, 160, 160), cv::Scalar(255), -1);
    cv::circle(image, cv::Point(420, 260), 70, cv::Scalar(0), -1);
    auto keyframe = std::make_shared<KeyFrame>(image, 0.0, extractor, camera);
    keyframe->SetPose(SE3::Identity());
    return keyframe;
}

}  // namespace

TEST(LoopClosingTest, QueuesKeyframesAndCanRestartAfterStop) {
    const Camera camera = MakeCamera();
    Map map;

    features::OrbExtractor::Options extractor_options;
    extractor_options.num_features = 200;
    features::OrbExtractor extractor(extractor_options);

    auto keyframe = MakeKeyFrame(camera, extractor);
    map.AddKeyFrame(keyframe);

    LoopClosingConfig config;
    config.min_similarity_score = 0.71;
    config.min_consecutive_loops = 2;
    config.pose_graph_iterations = 7;
    config.enable_global_ba = false;

    loop_closing::LoopClosing loop_closing(map, camera, config);
    EXPECT_FALSE(loop_closing.IsRunning());
    EXPECT_FALSE(loop_closing.IsFinished());
    EXPECT_FALSE(loop_closing.LoadVocabulary(""));
    EXPECT_DOUBLE_EQ(loop_closing.DetectorSettings().min_score, config.min_similarity_score);
    EXPECT_EQ(loop_closing.DetectorSettings().min_consecutive, config.min_consecutive_loops);
    EXPECT_EQ(loop_closing.PoseGraphSettings().max_iterations, config.pose_graph_iterations);

    loop_closing.InsertKeyFrame(keyframe);
    EXPECT_EQ(loop_closing.QueueSize(), 1);

    loop_closing.Start();
    loop_closing.Stop();
    EXPECT_FALSE(loop_closing.IsRunning());
    EXPECT_TRUE(loop_closing.IsFinished());
    EXPECT_EQ(loop_closing.QueueSize(), 0);

    // A completed worker remains joinable until joined.  This used to cause
    // std::terminate when Start() assigned a replacement std::thread.
    EXPECT_NO_THROW(loop_closing.Start());
    EXPECT_NO_THROW(loop_closing.Stop());
    EXPECT_TRUE(loop_closing.IsFinished());
}

TEST(LoopClosingTest, MapperAndLoopWorkerSerializeSharedGraphAccess) {
    const Camera camera = MakeCamera();
    Map map;
    features::OrbExtractor::Options extractor_options;
    extractor_options.num_features = 200;
    features::OrbExtractor extractor(extractor_options);
    auto keyframe = MakeKeyFrame(camera, extractor);
    map.AddKeyFrame(keyframe);

    MappingConfig mapping_config;
    mapping::LocalMapper mapper(map, camera, mapping_config, extractor);
    LoopClosingConfig loop_config;
    loop_config.enable_global_ba = false;
    loop_closing::LoopClosing loop_closing(map, camera, loop_config);

    mapper.Start();
    loop_closing.Start();
    mapper.InsertKeyFrame(keyframe);
    loop_closing.InsertKeyFrame(keyframe);

    // Both workers use the same KeyFrame and Map. Stop drains their queued
    // work before joining, so this regression exercises the map graph lock
    // instead of merely starting idle threads.
    loop_closing.Stop();
    mapper.Stop();

    EXPECT_TRUE(loop_closing.IsFinished());
    EXPECT_TRUE(mapper.IsFinished());
    EXPECT_EQ(loop_closing.QueueSize(), 0);
    EXPECT_EQ(mapper.QueueSize(), 0);
}

TEST(GlobalBundleAdjusterTest, StopAndRestartAreSafe) {
    const Camera camera = MakeCamera();
    Map map;
    loop_closing::GlobalBundleAdjuster adjuster;
    loop_closing::GlobalBAConfig config;
    config.max_iterations = 1;

    EXPECT_FALSE(adjuster.IsRunning());
    EXPECT_FALSE(adjuster.IsFinished());
    EXPECT_NO_THROW(adjuster.Stop());

    // An empty map makes Ceres return immediately when available; this test
    // also covers the no-Ceres implementation used by the lightweight CI.
    EXPECT_NO_THROW(adjuster.Start(map, camera, config));
    EXPECT_NO_THROW(adjuster.Stop());
    EXPECT_FALSE(adjuster.IsRunning());
    EXPECT_TRUE(adjuster.IsFinished());

    EXPECT_NO_THROW(adjuster.Start(map, camera, config));
    EXPECT_NO_THROW(adjuster.Stop());
    EXPECT_FALSE(adjuster.IsRunning());
    EXPECT_TRUE(adjuster.IsFinished());
}

TEST(LoopCorrectorTest, DistributesWorldCoordinateCorrectionOverTemporalPath) {
    const Camera camera = MakeCamera();
    features::OrbExtractor::Options extractor_options;
    extractor_options.num_features = 200;
    features::OrbExtractor extractor(extractor_options);
    Map map;

    Frame::ResetIdCounter();
    auto matched = MakeKeyFrame(camera, extractor);
    auto middle = MakeKeyFrame(camera, extractor);
    auto current = MakeKeyFrame(camera, extractor);
    map.AddKeyFrame(matched);
    map.AddKeyFrame(middle);
    map.AddKeyFrame(current);

    SE3 matched_pose = SE3::Identity();
    matched->SetPose(matched_pose);
    SE3 middle_pose = SE3::Identity();
    middle_pose.translation() = Vec3(-1.0, 0.0, 0.0);
    middle->SetPose(middle_pose);
    SE3 current_pose = SE3::Identity();
    current_pose.translation() = Vec3(-2.0, 0.0, 0.0);
    current->SetPose(current_pose);

    geometry::Sim3 correction;
    correction.t = Vec3(-2.0, 0.0, 0.0);
    loop_closing::LoopCorrector corrector;
    corrector.CorrectLoop(current, matched, correction, {}, {}, {}, map);

    // The old end point at world x=2 receives the full -2 correction.  The
    // middle receives half, while the matched anchor remains fixed.
    EXPECT_NEAR(matched->CameraCenter().x(), 0.0, 1e-12);
    EXPECT_NEAR(middle->CameraCenter().x(), 0.0, 1e-12);
    EXPECT_NEAR(current->CameraCenter().x(), 0.0, 1e-12);
}

TEST(LoopCorrectorTest, KeepsMatchedNeighborhoodFixedBeforeSmoothCorrection) {
    loop_closing::LoopCorrection correction;
    correction.matched_frame = FrameId{100};
    correction.current_frame = FrameId{2100};
    correction.transform.t = Vec3(8.0, -2.0, 1.0);
    correction.transform.s = 2.0;

    // A quarter of this 2000-frame loop is retained as a locally consistent
    // anchor. The correction then ramps smoothly and reaches the exact target
    // at the current frame.
    const geometry::Sim3 anchored =
        loop_closing::InterpolateLoopCorrection(correction, FrameId{600});
    EXPECT_NEAR(anchored.s, 1.0, 1e-12);
    EXPECT_NEAR(anchored.t.norm(), 0.0, 1e-12);

    const geometry::Sim3 ramped =
        loop_closing::InterpolateLoopCorrection(correction, FrameId{1100});
    EXPECT_GT(ramped.s, 1.0);
    EXPECT_LT(ramped.s, correction.transform.s);
    EXPECT_GT(ramped.t.norm(), 0.0);
    EXPECT_LT(ramped.t.norm(), correction.transform.t.norm());

    const geometry::Sim3 completed =
        loop_closing::InterpolateLoopCorrection(correction, correction.current_frame);
    EXPECT_NEAR(completed.s, correction.transform.s, 1e-12);
    EXPECT_NEAR((completed.t - correction.transform.t).norm(), 0.0, 1e-12);
}

TEST(LoopCorrectorTest, FusionRedirectsEveryKeyFrameToSurvivingPoint) {
    const Camera camera = MakeCamera();
    features::OrbExtractor::Options extractor_options;
    extractor_options.num_features = 200;
    features::OrbExtractor extractor(extractor_options);
    Map map;
    MapPoint::ResetIdCounter();
    Frame::ResetIdCounter();

    auto matched = MakeKeyFrame(camera, extractor);
    auto unrelated = MakeKeyFrame(camera, extractor);
    auto current = MakeKeyFrame(camera, extractor);
    ASSERT_GT(current->NumKeyPoints(), 0);
    ASSERT_GT(matched->NumKeyPoints(), 0);
    ASSERT_GT(unrelated->NumKeyPoints(), 0);
    map.AddKeyFrame(matched);
    map.AddKeyFrame(unrelated);
    map.AddKeyFrame(current);

    auto absorbed = map.AddMapPoint(Vec3(0.0, 0.0, 5.0), current->Id());
    auto survivor = map.AddMapPoint(Vec3(0.01, 0.0, 5.0), matched->Id());
    const cv::Mat descriptor = cv::Mat::zeros(1, 32, CV_8UC1);
    absorbed->SetDescriptor(descriptor);
    survivor->SetDescriptor(descriptor);
    absorbed->AddObservation(current->Id());
    survivor->AddObservation(matched->Id());
    survivor->AddObservation(unrelated->Id());

    current->SetMapPointId(0, absorbed->Id());
    matched->SetMapPointId(0, survivor->Id());
    // This KF is outside loop_kfs; it used to retain a bad-point association.
    unrelated->SetMapPointId(0, absorbed->Id());

    loop_closing::LoopCorrector corrector;
    corrector.CorrectLoop(current, matched, geometry::Sim3::Identity(), {}, {absorbed->Id()},
                          {survivor->Id()}, map);

    EXPECT_TRUE(absorbed->IsBad());
    EXPECT_FALSE(survivor->IsBad());
    EXPECT_EQ(current->MapPointIdAt(0), survivor->Id());
    EXPECT_EQ(matched->MapPointIdAt(0), survivor->Id());
    EXPECT_EQ(unrelated->MapPointIdAt(0), survivor->Id());
}

}  // namespace slamforge
