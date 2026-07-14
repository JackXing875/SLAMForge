// =============================================================================
// LiteVO epipolar geometry — implementation
// =============================================================================

#include "litevo/geometry/epipolar.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace litevo::geometry {

EpipolarResult EstimatePoseEssential(
    const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2,
    const cv::Mat& K,
    const EpipolarOptions& options) {

    EpipolarResult result;

    if (static_cast<int>(pts1.size()) < options.min_points ||
        static_cast<int>(pts2.size()) < options.min_points) {
        return result;
    }

    if (pts1.size() != pts2.size()) {
        return result;
    }

    // Estimate essential matrix with MAGSAC++ for robust outlier rejection
    cv::Mat inlier_mask;
    cv::Mat E = cv::findEssentialMat(
        pts1, pts2, K,
        cv::USAC_MAGSAC,
        options.confidence,
        options.epipolar_threshold,
        inlier_mask
    );

    if (E.empty() || E.rows != 3 || E.cols != 3) {
        return result;
    }

    // Recover pose from essential matrix
    cv::Mat R_cv, t_cv;
    cv::recoverPose(E, pts1, pts2, K, R_cv, t_cv, inlier_mask);

    if (R_cv.empty() || t_cv.empty()) {
        return result;
    }

    // Count inliers and collect indices
    int inlier_count = 0;
    for (int i = 0; i < inlier_mask.rows; ++i) {
        if (inlier_mask.at<uchar>(i)) {
            result.inlier_indices.push_back(i);
            ++inlier_count;
        }
    }

    // Convert to Eigen types
    R_cv.convertTo(R_cv, CV_64F);
    t_cv.convertTo(t_cv, CV_64F);

    cv::cv2eigen(R_cv, result.R);
    cv::cv2eigen(t_cv, result.t);

    result.num_inliers = inlier_count;
    result.valid = (inlier_count >= options.min_points);

    return result;
}

int RecoverPose(const cv::Mat& E,
                const std::vector<cv::Point2f>& pts1,
                const std::vector<cv::Point2f>& pts2,
                const cv::Mat& K,
                cv::Mat& R_out,
                cv::Mat& t_out,
                cv::Mat& mask_inout) {
    return cv::recoverPose(E, pts1, pts2, K, R_out, t_out, mask_inout);
}

Mat3 FundamentalFromEssential(const Mat3& E, const Mat3& K) {
    Mat3 K_inv = K.inverse();
    return K_inv.transpose() * E * K_inv;
}

}  // namespace litevo::geometry
