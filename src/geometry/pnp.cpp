// =============================================================================
// SLAMForge PnP solver — implementation
// =============================================================================

#include "slamforge/geometry/pnp.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "slamforge/geometry/se3.h"

namespace slamforge::geometry {

PnPResult SolvePnPRansac(const std::vector<cv::Point3f>& points_3d,
                         const std::vector<cv::Point2f>& points_2d, const cv::Mat& K,
                         const PnPOptions& options, const SE3* initial_pose) {
    PnPResult result;

    if (points_3d.size() < 4 || points_2d.size() < 4 || points_3d.size() != points_2d.size()) {
        return result;
    }

    cv::Mat rvec, tvec, inliers;
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

    const bool use_guess = options.use_extrinsic_guess && initial_pose != nullptr;
    if (use_guess) {
        ToOpenCVRt(*initial_pose, rvec, tvec);
    }

    const bool ok =
        cv::solvePnPRansac(points_3d, points_2d, K, dist_coeffs, rvec, tvec, use_guess,
                           options.max_iterations, static_cast<float>(options.max_reproj_error),
                           options.confidence, inliers, cv::SOLVEPNP_ITERATIVE);

    if (!ok || inliers.rows < 4) {
        return result;
    }

    result.T_cw = FromOpenCVRt(rvec, tvec);

    result.inlier_indices.reserve(static_cast<size_t>(inliers.rows));
    for (int i = 0; i < inliers.rows; ++i) {
        result.inlier_indices.push_back(inliers.at<int>(i));
    }
    result.num_inliers = static_cast<int>(result.inlier_indices.size());
    result.valid = true;

    return result;
}

bool RefinePnP(const std::vector<cv::Point3f>& points_3d, const std::vector<cv::Point2f>& points_2d,
               const cv::Mat& K, SE3& T_cw) {
    if (points_3d.size() < 4 || points_2d.size() < 4 || points_3d.size() != points_2d.size()) {
        return false;
    }

    cv::Mat rvec, tvec;
    ToOpenCVRt(T_cw, rvec, tvec);
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

    const bool ok = cv::solvePnP(points_3d, points_2d, K, dist_coeffs, rvec, tvec,
                                 true,  // useExtrinsicGuess
                                 cv::SOLVEPNP_ITERATIVE);

    if (ok) {
        T_cw = FromOpenCVRt(rvec, tvec);
    }

    return ok;
}

SE3 FromOpenCVRt(const cv::Mat& rvec, const cv::Mat& tvec) {
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    Mat3 R;
    Vec3 t;
    cv::cv2eigen(R_cv, R);
    cv::cv2eigen(tvec, t);

    return MakeSE3(R, t);
}

void ToOpenCVRt(const SE3& T_cw, cv::Mat& rvec, cv::Mat& tvec) {
    Mat3 R = T_cw.rotation();
    Vec3 t = T_cw.translation();

    cv::Mat R_cv;
    cv::eigen2cv(R, R_cv);
    cv::Rodrigues(R_cv, rvec);
    cv::eigen2cv(t, tvec);
}

}  // namespace slamforge::geometry
