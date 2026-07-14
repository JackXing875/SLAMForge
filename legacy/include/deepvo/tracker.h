#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

#include "deepvo/geometry/epipolar.h"

namespace deepvo {
namespace tracker {

class VisualOdometryTracker {
 public:
    explicit VisualOdometryTracker(const cv::Mat& K);

    bool ProcessFrame(const cv::Mat& frame);

    cv::Mat GetCurrentR() const;
    cv::Mat GetCurrentT() const;
    cv::Mat GetCurrentPosition() const;
    std::vector<cv::Point2f> GetTrackedPoints() const;
    std::string GetStatusText() const;
    bool HasActivePose() const;
    int GetVisibleTrackCount() const;
    int GetLandmarkCount() const;

 private:
    enum class Stage {
        kBootstrap,
        kTracking,
    };

    struct Landmark {
        cv::Point3f position_w;
        int observations = 0;
    };

    static constexpr int kBootstrapMinFeatures = 300;
    static constexpr int kTrackingMinFeatures = 80;
    static constexpr int kTargetActiveTracks = 320;
    static constexpr int kMaxFeatureCount = 1200;
    static constexpr int kMinLandmarksForTracking = 20;
    static constexpr int kPnPMinInliers = 25;
    static constexpr double kMinFeatureDistance = 12.0;
    static constexpr double kBootstrapMinParallaxPixels = 18.0;
    static constexpr double kMinTriangulationParallaxDeg = 1.5;
    static constexpr double kMaxReprojectionErrorPx = 3.0;
    static constexpr double kPnPMaxReprojectionErrorPx = 3.0;

    geometry::EpipolarGeometry geo_solver_;
    Stage stage_;

    cv::Mat K_;
    cv::Mat current_R_;
    cv::Mat current_t_;
    cv::Mat prev_pose_R_;
    cv::Mat prev_pose_t_;

    cv::Mat prev_frame_gray_;
    std::vector<cv::Point2f> prev_kpts_;
    std::vector<int> prev_landmark_ids_;

    cv::Mat bootstrap_frame_gray_;
    std::vector<cv::Point2f> bootstrap_kpts_;

    std::unordered_map<int, Landmark> landmarks_;
    int next_landmark_id_;
    std::string status_text_;

    void ResetToBootstrap();
    void StartBootstrap(const cv::Mat& frame_gray);
    bool TryBootstrap(const cv::Mat& frame_gray);
    bool TrackFrame(const cv::Mat& frame_gray);

    void DetectNewFeatures(const cv::Mat& frame_gray,
                           const std::vector<cv::Point2f>& occupied_points,
                           std::vector<cv::Point2f>& kpts,
                           int max_corners) const;

    bool TrackPoints(const cv::Mat& prev_gray, const cv::Mat& cur_gray,
                     const std::vector<cv::Point2f>& prev_pts,
                     std::vector<cv::Point2f>& cur_pts,
                     std::vector<uchar>& valid_status) const;

    int AppendTriangulatedLandmarks(const std::vector<cv::Point2f>& pts1,
                                    const std::vector<cv::Point2f>& pts2,
                                    const cv::Mat& R1, const cv::Mat& t1,
                                    const cv::Mat& R2, const cv::Mat& t2,
                                    std::vector<cv::Point2f>& accepted_cur_points,
                                    std::vector<int>& accepted_landmark_ids);

    void AddLandmarksFromMotion(const cv::Mat& prev_gray, const cv::Mat& cur_gray,
                                const cv::Mat& prev_R, const cv::Mat& prev_t,
                                const cv::Mat& cur_R, const cv::Mat& cur_t,
                                const std::vector<cv::Point2f>& occupied_prev,
                                std::vector<cv::Point2f>& cur_points,
                                std::vector<int>& cur_ids);

    cv::Mat ComposeProjectionMatrix(const cv::Mat& R, const cv::Mat& t) const;
    cv::Mat CameraPositionFromPose(const cv::Mat& R, const cv::Mat& t) const;

    static bool IsInsideImage(const cv::Point2f& pt, const cv::Size& size, int border = 8);
    static double ComputeMedian(std::vector<double> values);
};

}  // namespace tracker
}  // namespace deepvo
