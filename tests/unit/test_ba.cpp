// =============================================================================
// Bundle Adjustment unit tests
// =============================================================================

#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "slamforge/core/camera.h"
#include "slamforge/core/frame.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"
#include "slamforge/features/orb_extractor.h"
#include "slamforge/loop_closing/global_ba.h"
#include "slamforge/optimization/ba.h"

using namespace slamforge;

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

#ifdef SLAMFORGE_HAS_CERES

namespace {

// Exact observations isolate the optimizer from feature-detection and matching
// noise. Only this fixture replaces feature coordinates; BA does not use the
// frame's spatial lookup grid. No production feature mutator is needed.
void SetObservation(KeyFrame& frame, int index, const Camera& camera, const Vec3& point_camera) {
    const Vec2 distorted = camera.Project(point_camera);
    const Vec2 undistorted = camera.ProjectUndistorted(point_camera);
    auto& raw = const_cast<std::vector<cv::KeyPoint>&>(frame.KeyPoints());
    auto& corrected = const_cast<std::vector<cv::KeyPoint>&>(frame.KeyPointsUndistorted());
    raw.at(static_cast<size_t>(index)).pt =
        cv::Point2f(static_cast<float>(distorted.x()), static_cast<float>(distorted.y()));
    corrected.at(static_cast<size_t>(index)).pt =
        cv::Point2f(static_cast<float>(undistorted.x()), static_cast<float>(undistorted.y()));
}

class NumericalBundleAdjustmentTest : public ::testing::Test {
protected:
    void SetUp() override {
        Frame::ResetIdCounter();
        MapPoint::ResetIdCounter();
        camera_ = Camera(500.0, 500.0, 320.0, 240.0, 640, 480);
        camera_.SetDistortion(0.18, -0.03, 0.003, -0.002, 0.01);

        cv::Mat image(480, 640, CV_8UC1);
        cv::RNG random(0x7193);
        random.fill(image, cv::RNG::UNIFORM, 0, 256);
        features::OrbExtractor extractor;
        for (int i = 0; i < 4; ++i) {
            auto kf = std::make_shared<KeyFrame>(image, i, extractor, camera_);
            ASSERT_GE(kf->NumKeyPoints(), 84);
            SE3 pose = SE3::Identity();
            pose.linear() = Eigen::AngleAxisd(0.015 * i, Vec3::UnitY()).toRotationMatrix();
            pose.translation() = -pose.rotation() * Vec3(0.25 * i, 0.02 * i * i, 0.01 * i);
            kf->SetPose(pose);
            truth_.push_back(pose);
            keyframes_.push_back(kf);
            map_.AddKeyFrame(kf);
        }
        for (int i = 0; i < 80; ++i) {
            const Vec3 point(0.35 * (i % 8 - 3.5), 0.22 * (i / 8 - 4.5), 4.0 + 0.2 * (i % 7));
            auto mp = map_.AddMapPoint(point, keyframes_.front()->Id());
            for (size_t j = 0; j < keyframes_.size(); ++j) {
                SetObservation(*keyframes_[j], i, camera_, truth_[j] * point);
                keyframes_[j]->SetMapPointId(i, mp->Id());
                mp->AddObservation(keyframes_[j]->Id());
            }
            points_.push_back(mp);
        }
    }

    void PerturbLocalPoses() {
        for (size_t i = 2; i < keyframes_.size(); ++i) {
            SE3 pose = truth_[i];
            pose.linear() =
                Eigen::AngleAxisd(0.025, Vec3::UnitX()).toRotationMatrix() * pose.rotation();
            pose.translation() += Vec3(0.08, -0.06, 0.04);
            keyframes_[i]->SetPose(pose);
        }
    }

    double PoseError(size_t index) const {
        return (keyframes_[index]->Pose().matrix() - truth_[index].matrix()).norm();
    }

    double ReprojectionRMSE() const {
        double error = 0.0;
        size_t count = 0;
        for (const auto& kf : keyframes_) {
            for (size_t i = 0; i < points_.size(); ++i) {
                const auto pixel = kf->KeyPointsUndistorted()[i].pt;
                const Vec2 predicted =
                    camera_.ProjectWorldUndistorted(points_[i]->Position(), kf->Pose());
                error += (predicted - Vec2(pixel.x, pixel.y)).squaredNorm();
                ++count;
            }
        }
        return std::sqrt(error / static_cast<double>(count));
    }

    void RunGlobalBA() {
        loop_closing::GlobalBAConfig config;
        config.max_iterations = 50;
        loop_closing::GlobalBundleAdjuster adjuster;
        adjuster.Start(map_, camera_, config);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!adjuster.IsFinished() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const bool finished = adjuster.IsFinished();
        adjuster.Stop();
        ASSERT_TRUE(finished);
        EXPECT_GT(adjuster.Iterations(), 0);
        EXPECT_TRUE(std::isfinite(adjuster.FinalCost()));
    }

    Camera camera_;
    Map map_;
    std::vector<SE3> truth_;
    std::vector<std::shared_ptr<KeyFrame>> keyframes_;
    std::vector<std::shared_ptr<MapPoint>> points_;
};

TEST_F(NumericalBundleAdjustmentTest, LocalBARefinesEveryLocalPoseWithFixedBaseline) {
    PerturbLocalPoses();
    const double initial_error = ReprojectionRMSE();
    std::vector<KeyFrame*> local{keyframes_[2].get(), keyframes_[3].get()};
    std::vector<KeyFrame*> fixed{keyframes_[0].get(), keyframes_[1].get()};
    std::vector<MapPoint*> points;
    for (const auto& point : points_) {
        points.push_back(point.get());
    }
    optimization::BAOptions options;
    options.max_iterations = 50;
    optimization::LocalBundleAdjuster adjuster(camera_, options);
    adjuster.Optimize(local, fixed, points);

    EXPECT_LT(ReprojectionRMSE(), initial_error * 0.001);
    EXPECT_LT(PoseError(2), 1e-4);
    EXPECT_LT(PoseError(3), 1e-4);
    EXPECT_EQ(PoseError(0), 0.0);
    EXPECT_EQ(PoseError(1), 0.0);
    RecordProperty("initial_reprojection_rmse_px", initial_error);
    RecordProperty("final_reprojection_rmse_px", ReprojectionRMSE());
}

TEST_F(NumericalBundleAdjustmentTest, GlobalBAPreservesCalibratedDistortedObservations) {
    ASSERT_LT(ReprojectionRMSE(), 1e-4);
    RunGlobalBA();

    EXPECT_LT(ReprojectionRMSE(), 1e-4);
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        EXPECT_LT(PoseError(i), 1e-4);
    }
    RecordProperty("final_reprojection_rmse_px", ReprojectionRMSE());
}

TEST_F(NumericalBundleAdjustmentTest, LocalBAIgnoresDuplicateAndOverlappingWindowEntries) {
    PerturbLocalPoses();
    std::vector<KeyFrame*> local{nullptr, keyframes_[2].get(), keyframes_[0].get(),
                                 keyframes_[2].get(), keyframes_[3].get()};
    std::vector<KeyFrame*> fixed{keyframes_[0].get(), nullptr, keyframes_[1].get(),
                                 keyframes_[0].get()};
    std::vector<MapPoint*> points{nullptr, points_.front().get()};
    for (const auto& point : points_) {
        points.push_back(point.get());
    }
    optimization::BAOptions options;
    options.max_iterations = 50;
    optimization::LocalBundleAdjuster adjuster(camera_, options);
    adjuster.Optimize(local, fixed, points);

    EXPECT_LT(ReprojectionRMSE(), 1e-4);
    EXPECT_LT(PoseError(2), 1e-4);
    EXPECT_EQ(PoseError(0), 0.0);
    EXPECT_EQ(PoseError(1), 0.0);
}

TEST_F(NumericalBundleAdjustmentTest, LocalBAKeepsGaugeWhenFixedCameraHasNoObservations) {
    for (int index = 0; index < keyframes_[1]->NumKeyPoints(); ++index) {
        keyframes_[1]->SetMapPointId(index, MapPointId{0});
    }
    PerturbLocalPoses();
    const Mat4 anchored_pose = keyframes_[2]->Pose().matrix();
    std::vector<KeyFrame*> local{keyframes_[2].get(), keyframes_[3].get()};
    std::vector<KeyFrame*> fixed{keyframes_[0].get(), keyframes_[1].get()};
    std::vector<MapPoint*> points;
    for (const auto& point : points_) {
        points.push_back(point.get());
    }
    optimization::LocalBundleAdjuster adjuster(camera_);
    adjuster.Optimize(local, fixed, points);

    EXPECT_EQ((keyframes_[2]->Pose().matrix() - anchored_pose).norm(), 0.0);
    EXPECT_EQ(PoseError(0), 0.0);
    EXPECT_EQ(PoseError(1), 0.0);
}

TEST_F(NumericalBundleAdjustmentTest, LocalBAAnchorsDisconnectedComponentsIndependently) {
    std::vector<MapPoint*> points;
    for (size_t index = 0; index < points_.size(); ++index) {
        points.push_back(points_[index].get());
        auto independent = map_.AddMapPoint(points_[index]->Position(), keyframes_[2]->Id());
        points.push_back(independent.get());
        for (size_t frame = 2; frame < keyframes_.size(); ++frame) {
            keyframes_[frame]->SetMapPointId(static_cast<int>(index), independent->Id());
            independent->AddObservation(keyframes_[frame]->Id());
        }
    }
    PerturbLocalPoses();
    const Mat4 oldest_pose = keyframes_[2]->Pose().matrix();
    const Mat4 second_pose = keyframes_[3]->Pose().matrix();
    std::vector<KeyFrame*> local{keyframes_[3].get(), keyframes_[2].get()};
    std::vector<KeyFrame*> fixed{keyframes_[0].get(), keyframes_[1].get()};
    optimization::LocalBundleAdjuster adjuster(camera_);
    adjuster.Optimize(local, fixed, points);

    EXPECT_EQ((keyframes_[2]->Pose().matrix() - oldest_pose).norm(), 0.0);
    EXPECT_EQ((keyframes_[3]->Pose().matrix() - second_pose).norm(), 0.0);
    EXPECT_EQ(PoseError(0), 0.0);
    EXPECT_EQ(PoseError(1), 0.0);
}

TEST_F(NumericalBundleAdjustmentTest, GlobalBAConvergesWithInvalidAndSingleViewLandmarks) {
    const Vec3 single_position(0.1, 0.2, 5.0);
    auto single_view = map_.AddMapPoint(single_position, keyframes_[0]->Id());
    keyframes_[0]->SetMapPointId(80, single_view->Id());
    single_view->AddObservation(keyframes_[0]->Id());
    // Even repeated associations in one camera do not supply triangulation.
    keyframes_[0]->SetMapPointId(81, single_view->Id());
    auto camera_plane = map_.AddMapPoint(Vec3::Zero(), keyframes_[0]->Id());
    auto nonfinite = map_.AddMapPoint(Vec3(std::numeric_limits<double>::quiet_NaN(), 0.0, 5.0),
                                      keyframes_[0]->Id());
    for (const auto& kf : {keyframes_[0], keyframes_[1]}) {
        kf->SetMapPointId(82, camera_plane->Id());
        camera_plane->AddObservation(kf->Id());
    }
    for (const auto& kf : keyframes_) {
        kf->SetMapPointId(83, nonfinite->Id());
        nonfinite->AddObservation(kf->Id());
    }
    PerturbLocalPoses();
    const double initial_error = ReprojectionRMSE();
    RunGlobalBA();

    EXPECT_LT(ReprojectionRMSE(), initial_error * 0.001);
    EXPECT_EQ((single_view->Position() - single_position).norm(), 0.0);
    EXPECT_EQ(camera_plane->Position().norm(), 0.0);
    EXPECT_TRUE(std::isnan(nonfinite->Position().x()));
}

TEST_F(NumericalBundleAdjustmentTest, GlobalBACancelledBeforeGraphLockDoesNotChangeMap) {
    PerturbLocalPoses();
    std::vector<Mat4> initial_poses;
    for (const auto& kf : keyframes_) {
        initial_poses.push_back(kf->Pose().matrix());
    }
    std::vector<Vec3> initial_points;
    for (const auto& point : points_) {
        initial_points.push_back(point->Position());
    }
    loop_closing::GlobalBundleAdjuster adjuster;
    auto graph_lock = map_.AcquireGraphLock();
    adjuster.Start(map_, camera_, {});
    adjuster.RequestStop();
    graph_lock.unlock();
    adjuster.Stop();

    EXPECT_TRUE(adjuster.IsFinished());
    EXPECT_EQ(adjuster.Iterations(), 0);
    for (size_t index = 0; index < keyframes_.size(); ++index) {
        EXPECT_EQ((keyframes_[index]->Pose().matrix() - initial_poses[index]).norm(), 0.0);
    }
    for (size_t index = 0; index < points_.size(); ++index) {
        EXPECT_EQ((points_[index]->Position() - initial_points[index]).norm(), 0.0);
    }
}

}  // namespace

#endif  // SLAMFORGE_HAS_CERES
