#include "deepvo/tracker.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>

namespace deepvo {
namespace tracker {

VisualOdometryTracker::VisualOdometryTracker(const cv::Mat& K)
    : geo_solver_(K),
      stage_(Stage::kBootstrap),
      K_(K.clone()),
      next_landmark_id_(0),
      status_text_("bootstrap: waiting for parallax") {
    current_R_ = cv::Mat::eye(3, 3, CV_64F);
    current_t_ = cv::Mat::zeros(3, 1, CV_64F);
    prev_pose_R_ = current_R_.clone();
    prev_pose_t_ = current_t_.clone();
}

bool VisualOdometryTracker::ProcessFrame(const cv::Mat& frame) {
    if (frame.empty()) {
        status_text_ = "input error: empty frame";
        return false;
    }

    cv::Mat frame_gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    } else {
        frame_gray = frame.clone();
    }

    if (stage_ == Stage::kBootstrap) {
        return TryBootstrap(frame_gray);
    }
    return TrackFrame(frame_gray);
}

cv::Mat VisualOdometryTracker::GetCurrentR() const {
    return current_R_.clone();
}

cv::Mat VisualOdometryTracker::GetCurrentT() const {
    return current_t_.clone();
}

cv::Mat VisualOdometryTracker::GetCurrentPosition() const {
    return CameraPositionFromPose(current_R_, current_t_);
}

std::vector<cv::Point2f> VisualOdometryTracker::GetTrackedPoints() const {
    if (stage_ == Stage::kTracking) {
        return prev_kpts_;
    }
    return bootstrap_kpts_;
}

std::string VisualOdometryTracker::GetStatusText() const {
    return status_text_;
}

bool VisualOdometryTracker::HasActivePose() const {
    return stage_ == Stage::kTracking;
}

int VisualOdometryTracker::GetVisibleTrackCount() const {
    return static_cast<int>(GetTrackedPoints().size());
}

int VisualOdometryTracker::GetLandmarkCount() const {
    return static_cast<int>(landmarks_.size());
}

void VisualOdometryTracker::ResetToBootstrap() {
    stage_ = Stage::kBootstrap;
    current_R_ = cv::Mat::eye(3, 3, CV_64F);
    current_t_ = cv::Mat::zeros(3, 1, CV_64F);
    prev_pose_R_ = current_R_.clone();
    prev_pose_t_ = current_t_.clone();
    prev_frame_gray_.release();
    prev_kpts_.clear();
    prev_landmark_ids_.clear();
    bootstrap_frame_gray_.release();
    bootstrap_kpts_.clear();
    landmarks_.clear();
    next_landmark_id_ = 0;
}

void VisualOdometryTracker::StartBootstrap(const cv::Mat& frame_gray) {
    bootstrap_frame_gray_ = frame_gray.clone();
    DetectNewFeatures(frame_gray, {}, bootstrap_kpts_, kMaxFeatureCount);

    std::ostringstream oss;
    oss << "bootstrap: seeded " << bootstrap_kpts_.size() << " features";
    status_text_ = oss.str();
}

void VisualOdometryTracker::DetectNewFeatures(const cv::Mat& frame_gray,
                                              const std::vector<cv::Point2f>& occupied_points,
                                              std::vector<cv::Point2f>& kpts,
                                              int max_corners) const {
    cv::Mat mask(frame_gray.size(), CV_8UC1, cv::Scalar(255));
    for (const auto& pt : occupied_points) {
        cv::circle(mask, pt, static_cast<int>(kMinFeatureDistance), cv::Scalar(0), cv::FILLED);
    }

    cv::goodFeaturesToTrack(
        frame_gray,
        kpts,
        max_corners,
        0.01,
        kMinFeatureDistance,
        mask,
        3,
        false,
        0.04
    );
}

bool VisualOdometryTracker::TrackPoints(const cv::Mat& prev_gray, const cv::Mat& cur_gray,
                                        const std::vector<cv::Point2f>& prev_pts,
                                        std::vector<cv::Point2f>& cur_pts,
                                        std::vector<uchar>& valid_status) const {
    cur_pts.clear();
    valid_status.clear();
    if (prev_pts.empty()) {
        return false;
    }

    std::vector<uchar> forward_status;
    std::vector<float> forward_error;
    cv::calcOpticalFlowPyrLK(
        prev_gray, cur_gray, prev_pts, cur_pts,
        forward_status, forward_error,
        cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01)
    );

    std::vector<cv::Point2f> backtracked_pts;
    std::vector<uchar> backward_status;
    std::vector<float> backward_error;
    cv::calcOpticalFlowPyrLK(
        cur_gray, prev_gray, cur_pts, backtracked_pts,
        backward_status, backward_error,
        cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01)
    );

    valid_status.assign(prev_pts.size(), 0);
    const cv::Size image_size = cur_gray.size();
    for (size_t i = 0; i < prev_pts.size(); ++i) {
        if (!forward_status[i] || !backward_status[i]) {
            continue;
        }
        if (!IsInsideImage(cur_pts[i], image_size)) {
            continue;
        }

        const double fb_error = cv::norm(prev_pts[i] - backtracked_pts[i]);
        if (fb_error > 1.5) {
            continue;
        }

        valid_status[i] = 1;
    }

    return true;
}

bool VisualOdometryTracker::TryBootstrap(const cv::Mat& frame_gray) {
    if (bootstrap_frame_gray_.empty()) {
        StartBootstrap(frame_gray);
        return true;
    }

    std::vector<cv::Point2f> tracked_kpts;
    std::vector<uchar> valid_status;
    TrackPoints(bootstrap_frame_gray_, frame_gray, bootstrap_kpts_, tracked_kpts, valid_status);

    std::vector<cv::Point2f> matched_prev;
    std::vector<cv::Point2f> matched_cur;
    std::vector<double> disparities;
    matched_prev.reserve(bootstrap_kpts_.size());
    matched_cur.reserve(bootstrap_kpts_.size());
    disparities.reserve(bootstrap_kpts_.size());

    for (size_t i = 0; i < bootstrap_kpts_.size(); ++i) {
        if (!valid_status.empty() && !valid_status[i]) {
            continue;
        }
        matched_prev.push_back(bootstrap_kpts_[i]);
        matched_cur.push_back(tracked_kpts[i]);
        disparities.push_back(cv::norm(tracked_kpts[i] - bootstrap_kpts_[i]));
    }

    if (matched_cur.size() < static_cast<size_t>(kMinLandmarksForTracking)) {
        StartBootstrap(frame_gray);
        status_text_ = "bootstrap: refresh after track loss";
        return true;
    }

    const double median_parallax = ComputeMedian(std::move(disparities));
    if (matched_cur.size() < static_cast<size_t>(kBootstrapMinFeatures / 2) ||
        median_parallax < kBootstrapMinParallaxPixels) {
        std::ostringstream oss;
        oss << "bootstrap: waiting for motion (" << matched_cur.size() << " tracks)";
        status_text_ = oss.str();
        return true;
    }

    cv::Mat R, t, inlier_mask;
    std::tie(R, t, inlier_mask) = geo_solver_.EstimatePose(matched_prev, matched_cur);
    if (R.empty() || t.empty() || cv::countNonZero(inlier_mask) < kPnPMinInliers) {
        StartBootstrap(frame_gray);
        status_text_ = "bootstrap: pose init failed, reseeded";
        return true;
    }

    R.convertTo(current_R_, CV_64F);
    t.convertTo(current_t_, CV_64F);

    std::vector<cv::Point2f> inlier_prev;
    std::vector<cv::Point2f> inlier_cur;
    inlier_prev.reserve(matched_prev.size());
    inlier_cur.reserve(matched_cur.size());
    for (int i = 0; i < inlier_mask.rows; ++i) {
        if (inlier_mask.at<uchar>(i)) {
            inlier_prev.push_back(matched_prev[i]);
            inlier_cur.push_back(matched_cur[i]);
        }
    }

    std::vector<cv::Point2f> current_points;
    std::vector<int> current_ids;
    const cv::Mat identity_R = cv::Mat::eye(3, 3, CV_64F);
    const cv::Mat zero_t = cv::Mat::zeros(3, 1, CV_64F);
    const int created_landmarks = AppendTriangulatedLandmarks(
        inlier_prev, inlier_cur,
        identity_R, zero_t,
        current_R_, current_t_,
        current_points, current_ids
    );

    if (created_landmarks < kMinLandmarksForTracking) {
        StartBootstrap(frame_gray);
        status_text_ = "bootstrap: triangulation too weak, reseeded";
        return true;
    }

    stage_ = Stage::kTracking;
    prev_frame_gray_ = frame_gray.clone();
    prev_kpts_ = current_points;
    prev_landmark_ids_ = current_ids;
    prev_pose_R_ = current_R_.clone();
    prev_pose_t_ = current_t_.clone();
    bootstrap_frame_gray_.release();
    bootstrap_kpts_.clear();

    std::ostringstream oss;
    oss << "tracking: initialized with " << current_points.size() << " landmarks";
    status_text_ = oss.str();

    return true;
}

bool VisualOdometryTracker::TrackFrame(const cv::Mat& frame_gray) {
    if (prev_kpts_.size() < static_cast<size_t>(kMinLandmarksForTracking)) {
        ResetToBootstrap();
        StartBootstrap(frame_gray);
        status_text_ = "bootstrap: insufficient active landmarks";
        return true;
    }

    std::vector<cv::Point2f> tracked_kpts;
    std::vector<uchar> valid_status;
    TrackPoints(prev_frame_gray_, frame_gray, prev_kpts_, tracked_kpts, valid_status);

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    std::vector<int> tracked_ids;
    object_points.reserve(prev_kpts_.size());
    image_points.reserve(prev_kpts_.size());
    tracked_ids.reserve(prev_kpts_.size());

    for (size_t i = 0; i < prev_kpts_.size(); ++i) {
        if (!valid_status.empty() && !valid_status[i]) {
            continue;
        }

        const auto landmark_it = landmarks_.find(prev_landmark_ids_[i]);
        if (landmark_it == landmarks_.end()) {
            continue;
        }

        object_points.push_back(landmark_it->second.position_w);
        image_points.push_back(tracked_kpts[i]);
        tracked_ids.push_back(prev_landmark_ids_[i]);
    }

    if (object_points.size() < static_cast<size_t>(kMinLandmarksForTracking)) {
        ResetToBootstrap();
        StartBootstrap(frame_gray);
        status_text_ = "bootstrap: tracking dropped below recoverable limit";
        return true;
    }

    cv::Mat rvec;
    cv::Rodrigues(prev_pose_R_, rvec);
    cv::Mat tvec = prev_pose_t_.clone();
    cv::Mat inliers;

    const bool pnp_ok = cv::solvePnPRansac(
        object_points, image_points, K_, cv::noArray(),
        rvec, tvec, true,
        150,
        kPnPMaxReprojectionErrorPx,
        0.999,
        inliers,
        cv::SOLVEPNP_ITERATIVE
    );

    if (!pnp_ok || inliers.rows < kPnPMinInliers) {
        ResetToBootstrap();
        StartBootstrap(frame_gray);
        status_text_ = "bootstrap: PnP failed, reseeded";
        return true;
    }

    std::vector<cv::Point3f> inlier_object_points;
    std::vector<cv::Point2f> inlier_image_points;
    std::vector<int> inlier_ids;
    inlier_object_points.reserve(inliers.rows);
    inlier_image_points.reserve(inliers.rows);
    inlier_ids.reserve(inliers.rows);

    for (int i = 0; i < inliers.rows; ++i) {
        const int idx = inliers.at<int>(i);
        inlier_object_points.push_back(object_points[idx]);
        inlier_image_points.push_back(image_points[idx]);
        inlier_ids.push_back(tracked_ids[idx]);
    }

    cv::solvePnP(
        inlier_object_points, inlier_image_points, K_, cv::noArray(),
        rvec, tvec, true, cv::SOLVEPNP_ITERATIVE
    );

    cv::Rodrigues(rvec, current_R_);
    current_t_ = tvec.clone();

    std::vector<cv::Point2f> refined_points = inlier_image_points;
    std::vector<int> refined_ids = inlier_ids;
    for (const int id : refined_ids) {
        auto landmark_it = landmarks_.find(id);
        if (landmark_it != landmarks_.end()) {
            landmark_it->second.observations += 1;
        }
    }

    AddLandmarksFromMotion(
        prev_frame_gray_, frame_gray,
        prev_pose_R_, prev_pose_t_,
        current_R_, current_t_,
        prev_kpts_,
        refined_points, refined_ids
    );

    if (refined_points.size() < static_cast<size_t>(kTrackingMinFeatures)) {
        AddLandmarksFromMotion(
            prev_frame_gray_, frame_gray,
            prev_pose_R_, prev_pose_t_,
            current_R_, current_t_,
            {},
            refined_points, refined_ids
        );
    }

    prev_frame_gray_ = frame_gray.clone();
    prev_kpts_ = refined_points;
    prev_landmark_ids_ = refined_ids;
    prev_pose_R_ = current_R_.clone();
    prev_pose_t_ = current_t_.clone();

    std::ostringstream oss;
    oss << "tracking: " << refined_points.size()
        << " active / " << landmarks_.size() << " map points";
    status_text_ = oss.str();

    return true;
}

int VisualOdometryTracker::AppendTriangulatedLandmarks(
    const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2,
    const cv::Mat& R1, const cv::Mat& t1,
    const cv::Mat& R2, const cv::Mat& t2,
    std::vector<cv::Point2f>& accepted_cur_points,
    std::vector<int>& accepted_landmark_ids) {
    if (pts1.size() < static_cast<size_t>(kMinLandmarksForTracking) || pts1.size() != pts2.size()) {
        return 0;
    }

    cv::Mat points_4d;
    const cv::Mat P1 = ComposeProjectionMatrix(R1, t1);
    const cv::Mat P2 = ComposeProjectionMatrix(R2, t2);
    cv::triangulatePoints(P1, P2, pts1, pts2, points_4d);
    points_4d.convertTo(points_4d, CV_64F);

    const cv::Mat C1 = CameraPositionFromPose(R1, t1);
    const cv::Mat C2 = CameraPositionFromPose(R2, t2);

    auto project_error = [this](const cv::Mat& R, const cv::Mat& t,
                                const cv::Vec3d& point_w,
                                const cv::Point2f& measurement) {
        cv::Mat X = (cv::Mat_<double>(3, 1) << point_w[0], point_w[1], point_w[2]);
        cv::Mat x_cam = R * X + t;
        const double z = x_cam.at<double>(2);
        if (z <= 1e-6) {
            return std::numeric_limits<double>::infinity();
        }

        const double fx = K_.at<double>(0, 0);
        const double fy = K_.at<double>(1, 1);
        const double cx = K_.at<double>(0, 2);
        const double cy = K_.at<double>(1, 2);
        const double u = fx * x_cam.at<double>(0) / z + cx;
        const double v = fy * x_cam.at<double>(1) / z + cy;
        return std::hypot(u - measurement.x, v - measurement.y);
    };

    int appended = 0;
    for (int col = 0; col < points_4d.cols; ++col) {
        const double w = points_4d.at<double>(3, col);
        if (std::abs(w) < 1e-9) {
            continue;
        }

        const cv::Vec3d point_w(
            points_4d.at<double>(0, col) / w,
            points_4d.at<double>(1, col) / w,
            points_4d.at<double>(2, col) / w
        );

        if (!std::isfinite(point_w[0]) || !std::isfinite(point_w[1]) || !std::isfinite(point_w[2])) {
            continue;
        }

        cv::Mat X = (cv::Mat_<double>(3, 1) << point_w[0], point_w[1], point_w[2]);
        const cv::Mat x1 = R1 * X + t1;
        const cv::Mat x2 = R2 * X + t2;
        if (x1.at<double>(2) <= 0.0 || x2.at<double>(2) <= 0.0) {
            continue;
        }

        cv::Vec3d ray1(
            point_w[0] - C1.at<double>(0),
            point_w[1] - C1.at<double>(1),
            point_w[2] - C1.at<double>(2)
        );
        cv::Vec3d ray2(
            point_w[0] - C2.at<double>(0),
            point_w[1] - C2.at<double>(1),
            point_w[2] - C2.at<double>(2)
        );
        const double denom = cv::norm(ray1) * cv::norm(ray2);
        if (denom <= 1e-9) {
            continue;
        }

        double cos_angle = ray1.dot(ray2) / denom;
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        const double parallax_deg = std::acos(cos_angle) * 180.0 / CV_PI;
        if (parallax_deg < kMinTriangulationParallaxDeg) {
            continue;
        }

        const double reproj_err_1 = project_error(R1, t1, point_w, pts1[col]);
        const double reproj_err_2 = project_error(R2, t2, point_w, pts2[col]);
        if (reproj_err_1 > kMaxReprojectionErrorPx || reproj_err_2 > kMaxReprojectionErrorPx) {
            continue;
        }

        bool too_close = false;
        for (const auto& existing_pt : accepted_cur_points) {
            if (cv::norm(existing_pt - pts2[col]) < kMinFeatureDistance) {
                too_close = true;
                break;
            }
        }
        if (too_close) {
            continue;
        }

        const int landmark_id = next_landmark_id_++;
        landmarks_[landmark_id] = Landmark{
            cv::Point3f(static_cast<float>(point_w[0]),
                        static_cast<float>(point_w[1]),
                        static_cast<float>(point_w[2])),
            2
        };
        accepted_cur_points.push_back(pts2[col]);
        accepted_landmark_ids.push_back(landmark_id);
        ++appended;
    }

    return appended;
}

void VisualOdometryTracker::AddLandmarksFromMotion(
    const cv::Mat& prev_gray, const cv::Mat& cur_gray,
    const cv::Mat& prev_R, const cv::Mat& prev_t,
    const cv::Mat& cur_R, const cv::Mat& cur_t,
    const std::vector<cv::Point2f>& occupied_prev,
    std::vector<cv::Point2f>& cur_points,
    std::vector<int>& cur_ids) {
    if (cur_points.size() >= static_cast<size_t>(kTargetActiveTracks)) {
        return;
    }

    const cv::Mat prev_center = CameraPositionFromPose(prev_R, prev_t);
    const cv::Mat cur_center = CameraPositionFromPose(cur_R, cur_t);
    if (cv::norm(cur_center - prev_center) < 0.02) {
        return;
    }

    std::vector<cv::Point2f> candidate_prev;
    DetectNewFeatures(
        prev_gray,
        occupied_prev.empty() ? cur_points : occupied_prev,
        candidate_prev,
        kMaxFeatureCount - static_cast<int>(cur_points.size())
    );

    if (candidate_prev.size() < static_cast<size_t>(kMinLandmarksForTracking)) {
        return;
    }

    std::vector<cv::Point2f> candidate_cur;
    std::vector<uchar> candidate_status;
    TrackPoints(prev_gray, cur_gray, candidate_prev, candidate_cur, candidate_status);

    std::vector<cv::Point2f> filtered_prev;
    std::vector<cv::Point2f> filtered_cur;
    filtered_prev.reserve(candidate_prev.size());
    filtered_cur.reserve(candidate_prev.size());

    for (size_t i = 0; i < candidate_prev.size(); ++i) {
        if (!candidate_status.empty() && !candidate_status[i]) {
            continue;
        }

        bool duplicate = false;
        for (const auto& existing_pt : cur_points) {
            if (cv::norm(existing_pt - candidate_cur[i]) < kMinFeatureDistance) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }

        filtered_prev.push_back(candidate_prev[i]);
        filtered_cur.push_back(candidate_cur[i]);
    }

    AppendTriangulatedLandmarks(
        filtered_prev, filtered_cur,
        prev_R, prev_t,
        cur_R, cur_t,
        cur_points, cur_ids
    );
}

cv::Mat VisualOdometryTracker::ComposeProjectionMatrix(const cv::Mat& R, const cv::Mat& t) const {
    cv::Mat Rt;
    cv::hconcat(R, t, Rt);
    return K_ * Rt;
}

cv::Mat VisualOdometryTracker::CameraPositionFromPose(const cv::Mat& R, const cv::Mat& t) const {
    return -R.t() * t;
}

bool VisualOdometryTracker::IsInsideImage(const cv::Point2f& pt, const cv::Size& size, int border) {
    return pt.x >= border && pt.y >= border &&
           pt.x < static_cast<float>(size.width - border) &&
           pt.y < static_cast<float>(size.height - border);
}

double VisualOdometryTracker::ComputeMedian(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }

    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    double median = values[mid];
    if (values.size() % 2 == 0) {
        std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
        median = 0.5 * (median + values[mid - 1]);
    }
    return median;
}

}  // namespace tracker
}  // namespace deepvo
