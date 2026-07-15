// =============================================================================
// MonocularInitializer implementation
// =============================================================================

#include "litevo/tracking/initializer.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <thread>
#include <unordered_set>

#include "litevo/core/camera.h"
#include "litevo/core/frame.h"
#include "litevo/core/map_point.h"
#include "litevo/geometry/se3.h"
#include "litevo/geometry/triangulation.h"
#include "litevo/tracking/matcher.h"

namespace litevo::tracking {

MonocularInitializer::MonocularInitializer(const Camera& camera, features::OrbExtractor& extractor,
                                           const Options& opts)
    : camera_(camera), extractor_(extractor), opts_(opts) {}

void MonocularInitializer::Reset() {
    reference_frame_.reset();
}

void MonocularInitializer::ComputeModels(const std::vector<cv::Point2f>& pts1,
                                         const std::vector<cv::Point2f>& pts2, cv::Mat& H,
                                         cv::Mat& F, std::vector<uchar>& inliers_h,
                                         std::vector<uchar>& inliers_f) const {
    // Run H and F computation in parallel threads
    cv::Mat H_thread, F_thread;
    std::vector<uchar> inliers_h_thread, inliers_f_thread;

    auto compute_H = [&]() {
        H_thread = cv::findHomography(pts1, pts2, cv::RANSAC, opts_.max_reproj_error,
                                      inliers_h_thread, opts_.num_ransac_iterations);
    };

    auto compute_F = [&]() {
        F_thread = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, opts_.max_reproj_error, 0.99,
                                          inliers_f_thread);
    };

    std::thread h_thread(compute_H);
    compute_F();  // Run F in current thread
    h_thread.join();

    H = H_thread;
    F = F_thread;
    inliers_h = inliers_h_thread;
    inliers_f = inliers_f_thread;
}

bool MonocularInitializer::SelectModel(const cv::Mat& H, const cv::Mat& F,
                                       const std::vector<cv::Point2f>& pts1,
                                       const std::vector<cv::Point2f>& pts2,
                                       const std::vector<uchar>& inliers_h,
                                       const std::vector<uchar>& inliers_f) const {
    // Count inliers
    int n_inliers_h = 0, n_inliers_f = 0;
    for (auto v : inliers_h)
        if (v)
            n_inliers_h++;
    for (auto v : inliers_f)
        if (v)
            n_inliers_f++;

    if (n_inliers_h < opts_.min_matches && n_inliers_f < opts_.min_matches) {
        return false;  // Both models are bad
    }

    // Score the models with robust *inlier support*, rather than comparing
    // their mean residuals directly.  A residual is an error, so the former
    // implementation's SH / (SH + SF) selected the *worse* model whenever
    // both models were available.  The scoring below follows the ORB-SLAM
    // convention: every geometrically consistent observation contributes a
    // positive amount, and lower residuals contribute more.
    const double reprojection_threshold_sq = opts_.max_reproj_error * opts_.max_reproj_error;
    const auto score_residual = [reprojection_threshold_sq](double residual) {
        if (!std::isfinite(residual) || residual < 0.0 || residual > reprojection_threshold_sq) {
            return 0.0;
        }
        return reprojection_threshold_sq - residual;
    };

    // Homography: symmetric transfer error in both directions.
    double score_h = 0.0;
    if (!H.empty() && n_inliers_h >= opts_.min_matches) {
        cv::Mat H_inv = H.inv();
        for (size_t i = 0; i < pts1.size() && i < inliers_h.size(); ++i) {
            if (inliers_h[i]) {
                // Forward: x2' = H * x1
                cv::Mat p1 = (cv::Mat_<double>(3, 1) << pts1[i].x, pts1[i].y, 1.0);
                cv::Mat p2 = (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, 1.0);

                cv::Mat proj_fwd = H * p1;
                const double forward_w = proj_fwd.at<double>(2);
                if (std::abs(forward_w) > 1e-12) {
                    proj_fwd /= forward_w;
                    const double dx1 =
                        proj_fwd.at<double>(0) - static_cast<double>(pts2[i].x);
                    const double dy1 =
                        proj_fwd.at<double>(1) - static_cast<double>(pts2[i].y);
                    score_h += score_residual(dx1 * dx1 + dy1 * dy1);
                }

                cv::Mat proj_bwd = H_inv * p2;
                const double backward_w = proj_bwd.at<double>(2);
                if (std::abs(backward_w) > 1e-12) {
                    proj_bwd /= backward_w;
                    const double dx2 =
                        proj_bwd.at<double>(0) - static_cast<double>(pts1[i].x);
                    const double dy2 =
                        proj_bwd.at<double>(1) - static_cast<double>(pts1[i].y);
                    score_h += score_residual(dx2 * dx2 + dy2 * dy2);
                }
            }
        }
    }

    // Fundamental matrix: point-to-epipolar-line distance in both images.
    // This is comparable to the two directional homography residuals above.
    double score_f = 0.0;
    if (!F.empty() && n_inliers_f >= opts_.min_matches) {
        for (size_t i = 0; i < pts1.size() && i < inliers_f.size(); ++i) {
            if (inliers_f[i]) {
                double x1 = pts1[i].x, y1 = pts1[i].y;
                double x2 = pts2[i].x, y2 = pts2[i].y;

                cv::Mat p1 = (cv::Mat_<double>(3, 1) << x1, y1, 1.0);
                cv::Mat p2 = (cv::Mat_<double>(3, 1) << x2, y2, 1.0);

                cv::Mat Fp1 = F * p1;
                cv::Mat Ftp2 = F.t() * p2;

                double num = std::pow(p2.dot(Fp1), 2);
                const double den_forward = Fp1.at<double>(0) * Fp1.at<double>(0) +
                                           Fp1.at<double>(1) * Fp1.at<double>(1);
                const double den_backward = Ftp2.at<double>(0) * Ftp2.at<double>(0) +
                                            Ftp2.at<double>(1) * Ftp2.at<double>(1);

                if (den_forward > 1e-12) {
                    score_f += score_residual(num / den_forward);
                }
                if (den_backward > 1e-12) {
                    score_f += score_residual(num / den_backward);
                }
            }
        }
    }

    const double score_sum = score_h + score_f;
    if (score_sum <= 1e-12) {
        // Degenerate numeric case: retain the model with more RANSAC support.
        return n_inliers_h >= n_inliers_f;
    }

    const double homography_ratio = score_h / score_sum;
    return homography_ratio > opts_.hf_score_ratio;
}

std::vector<std::pair<Mat3, Vec3>> MonocularInitializer::DecomposeH(const cv::Mat& H,
                                                                    const cv::Mat& K) const {
    std::vector<std::pair<Mat3, Vec3>> candidates;

    if (H.empty())
        return candidates;

    // OpenCV expects the homography in pixel coordinates together with the
    // camera calibration matrix.  Passing a pre-normalized H along with K
    // applies the calibration twice and produces invalid motion candidates.
    std::vector<cv::Mat> rotations, translations, normals;
    int num_solutions = cv::decomposeHomographyMat(H, K, rotations, translations, normals);

    for (int i = 0; i < num_solutions; ++i) {
        Mat3 R;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                R(r, c) = rotations[static_cast<size_t>(i)].at<double>(r, c);

        Vec3 t;
        for (int r = 0; r < 3; ++r)
            t(r) = translations[static_cast<size_t>(i)].at<double>(r);

        candidates.emplace_back(R, t);
    }

    return candidates;
}

std::vector<std::pair<Mat3, Vec3>> MonocularInitializer::DecomposeE(
    const cv::Mat& E, const cv::Mat& K, const std::vector<cv::Point2f>& pts1_norm,
    const std::vector<cv::Point2f>& pts2_norm) const {
    std::vector<std::pair<Mat3, Vec3>> candidates;

    if (E.empty())
        return candidates;

    cv::Mat R1, R2, t;
    cv::decomposeEssentialMat(E, R1, R2, t);

    // 4 candidates: (R1, t), (R1, -t), (R2, t), (R2, -t)
    auto to_eigen = [](const cv::Mat& cv_mat) -> Mat3 {
        Mat3 m;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                m(r, c) = cv_mat.at<double>(r, c);
        return m;
    };

    Mat3 R1_e = to_eigen(R1);
    Mat3 R2_e = to_eigen(R2);
    Vec3 t_e(t.at<double>(0), t.at<double>(1), t.at<double>(2));

    candidates.emplace_back(R1_e, t_e);
    candidates.emplace_back(R1_e, -t_e);
    candidates.emplace_back(R2_e, t_e);
    candidates.emplace_back(R2_e, -t_e);

    return candidates;
}

int MonocularInitializer::CheckMotion(const Mat3& R, const Vec3& t,
                                      const std::vector<cv::Point2f>& pts1,
                                      const std::vector<cv::Point2f>& pts2, const cv::Mat& K,
                                      std::vector<Vec3>& points_3d,
                                      std::vector<bool>& valid) const {
    size_t n = pts1.size();
    points_3d.resize(n);
    valid.assign(n, false);

    // Build projection matrices
    // Camera 1 is identity (world frame)
    Mat34 P1 = Mat34::Zero();
    P1.block<3, 3>(0, 0) = Mat3::Identity();
    P1(0, 3) = P1(1, 3) = P1(2, 3) = 0.0;

    // Camera 2: [R | t]
    Mat34 P2 = Mat34::Zero();
    P2.block<3, 3>(0, 0) = R;
    P2.col(3) = t;

    // Apply intrinsic calibration
    Mat3 K_eigen;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            K_eigen(r, c) = K.at<double>(r, c);
    P1 = K_eigen * P1;
    P2 = K_eigen * P2;

    // Camera centers
    Vec3 C1(0, 0, 0);  // Camera 1 is at world origin
    Vec3 C2 = -R.transpose() * t;

    geometry::TriangulationOptions tri_opts;
    tri_opts.min_parallax_deg = opts_.min_parallax_deg;
    tri_opts.max_reprojection_px = opts_.max_reproj_error;
    tri_opts.min_depth = 0.01;

    int valid_count = 0;
    for (size_t i = 0; i < n; ++i) {
        Vec2 p1(pts1[i].x, pts1[i].y);
        Vec2 p2(pts2[i].x, pts2[i].y);

        auto result = geometry::TriangulatePoint(p1, p2, P1, P2, C1, C2, tri_opts);

        if (result.valid) {
            points_3d[i] = result.point_w;
            valid[i] = true;
            valid_count++;
        }
    }

    return valid_count;
}

InitializationResult MonocularInitializer::Initialize(Frame& frame) {
    InitializationResult result;

    if (!reference_frame_) {
        // First frame: store as reference
        if (frame.NumKeyPoints() < opts_.min_features) {
            return result;  // Not enough features
        }
        // Keep the exact feature set and frame identity that the tracker
        // supplied.  Re-extracting here changes keypoint indices, which made
        // initial map associations point at unrelated features.
        reference_frame_ = std::make_shared<Frame>(frame);
        return result;
    }

    // Second frame: try initialization
    if (frame.NumKeyPoints() < opts_.min_features) {
        Reset();
        return result;
    }

    // Match features
    FeatureMatcher matcher;
    std::vector<int> matches_12;
    int n_matches = matcher.SearchForInitialization(*reference_frame_, frame, 100, matches_12);

    if (n_matches < opts_.min_matches) {
        Reset();
        // Store current as new reference
        reference_frame_ = std::make_shared<Frame>(frame);
        return result;
    }

    // Collect matched points
    std::vector<cv::Point2f> pts1, pts2;
    std::vector<int> idx_ref, idx_cur;
    const auto& kps1 = reference_frame_->KeyPointsUndistorted();
    const auto& kps2 = frame.KeyPointsUndistorted();

    for (size_t i = 0; i < matches_12.size(); ++i) {
        if (matches_12[i] >= 0) {
            pts1.emplace_back(kps1[i].pt.x, kps1[i].pt.y);
            pts2.emplace_back(kps2[static_cast<size_t>(matches_12[i])].pt.x,
                              kps2[static_cast<size_t>(matches_12[i])].pt.y);
            idx_ref.push_back(static_cast<int>(i));
            idx_cur.push_back(matches_12[i]);
        }
    }

    // Build intrinsics matrix
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = camera_.fx();
    K.at<double>(1, 1) = camera_.fy();
    K.at<double>(0, 2) = camera_.cx();
    K.at<double>(1, 2) = camera_.cy();

    // Compute H and F in parallel
    cv::Mat H, F;
    std::vector<uchar> inliers_h, inliers_f;
    ComputeModels(pts1, pts2, H, F, inliers_h, inliers_f);

    // Select model
    bool use_H = SelectModel(H, F, pts1, pts2, inliers_h, inliers_f);

    // SelectModel uses false for the fundamental-matrix branch.  Do not try
    // to decompose an empty F when only a homography was estimated (or vice
    // versa).
    if (use_H && H.empty()) {
        if (F.empty()) {
            Reset();
            return result;
        }
        use_H = false;
    }
    if (!use_H && F.empty()) {
        if (H.empty()) {
            Reset();
            return result;
        }
        use_H = true;
    }

    // Get motion candidates
    std::vector<std::pair<Mat3, Vec3>> candidates;
    if (use_H) {
        candidates = DecomposeH(H, K);
    } else {
        // Convert pixels to normalized coordinates for E decomposition
        cv::Mat K_cv = K;
        std::vector<cv::Point2f> pts1_norm, pts2_norm;
        cv::Mat K_inv = K_cv.inv();
        for (size_t i = 0; i < pts1.size(); ++i) {
            cv::Mat p1 = (cv::Mat_<double>(3, 1) << pts1[i].x, pts1[i].y, 1.0);
            cv::Mat n1 = K_inv * p1;
            pts1_norm.emplace_back(static_cast<float>(n1.at<double>(0)),
                                   static_cast<float>(n1.at<double>(1)));

            cv::Mat p2 = (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, 1.0);
            cv::Mat n2 = K_inv * p2;
            pts2_norm.emplace_back(static_cast<float>(n2.at<double>(0)),
                                   static_cast<float>(n2.at<double>(1)));
        }

        // Convert F_cv (OpenCV Mat) to Eigen Mat3 for Essential matrix
        Mat3 F_eigen;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                F_eigen(r, c) = F.at<double>(r, c);

        Mat3 K_eigen;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                K_eigen(r, c) = K.at<double>(r, c);

        // E = K^T * F * K
        Mat3 E_eigen = K_eigen.transpose() * F_eigen * K_eigen;

        cv::Mat E_cv(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                E_cv.at<double>(r, c) = E_eigen(r, c);

        candidates = DecomposeE(E_cv, K_cv, pts1_norm, pts2_norm);
    }

    if (candidates.empty()) {
        Reset();
        return result;
    }

    // Test each candidate
    int best_valid = 0;
    std::vector<Vec3> best_points;
    std::vector<bool> best_valid_flags;
    Mat3 best_R = Mat3::Identity();
    Vec3 best_t = Vec3::Zero();

    for (const auto& [R, t] : candidates) {
        std::vector<Vec3> points;
        std::vector<bool> valid_flags;
        int n_valid = CheckMotion(R, t, pts1, pts2, K, points, valid_flags);

        if (n_valid > best_valid) {
            best_valid = n_valid;
            best_points = points;
            best_valid_flags = valid_flags;
            best_R = R;
            best_t = t;
        }
    }

    // Check minimum valid points
    if (best_valid < opts_.min_matches) {
        Reset();
        return result;
    }

    // Compute median depth for scale normalization
    std::vector<double> depths;
    for (size_t i = 0; i < best_points.size(); ++i) {
        if (best_valid_flags[i]) {
            // Depth in second camera
            Vec3 p_cam = best_R * best_points[i] + best_t;
            if (p_cam.z() > 0) {
                depths.push_back(p_cam.z());
            }
        }
    }

    double median_depth = 1.0;
    if (!depths.empty()) {
        std::sort(depths.begin(), depths.end());
        median_depth = depths[depths.size() / 2];
        if (median_depth < 1e-6)
            median_depth = 1.0;
    }

    // Scale translation and points
    double scale = 1.0 / median_depth;
    Vec3 t_scaled = scale * best_t;
    for (size_t i = 0; i < best_points.size(); ++i) {
        if (best_valid_flags[i]) {
            best_points[i] *= scale;
        }
    }

    // Build result
    result.success = true;
    result.Tcw = geometry::MakeSE3(best_R, t_scaled);
    result.model_used = use_H ? "Homography" : "Fundamental";
    result.reference_frame = reference_frame_;
    result.map_points.reserve(static_cast<size_t>(best_valid));
    result.match_indices_ref.reserve(static_cast<size_t>(best_valid));
    result.match_indices_cur.reserve(static_cast<size_t>(best_valid));

    // Keep feature indices parallel to map_points.  Some candidate matches
    // fail cheirality/reprojection checks, so returning all raw match indices
    // would shift later associations after the first rejected point.
    for (size_t i = 0; i < best_points.size(); ++i) {
        if (best_valid_flags[i]) {
            auto mp = std::make_shared<MapPoint>(best_points[i], reference_frame_->Id());
            // Set descriptor from reference frame keypoint
            mp->SetDescriptor(reference_frame_->Descriptors().row(idx_ref[i]));
            result.map_points.push_back(mp);
            result.match_indices_ref.push_back(idx_ref[i]);
            result.match_indices_cur.push_back(idx_cur[i]);
        }
    }

    // The result keeps the reference frame alive until Tracker has promoted
    // it to a keyframe; the initializer itself no longer needs it.
    reference_frame_.reset();

    return result;
}

}  // namespace litevo::tracking
