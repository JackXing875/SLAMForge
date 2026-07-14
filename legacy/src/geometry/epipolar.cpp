#include "deepvo/geometry/epipolar.h"

namespace deepvo {
namespace geometry {

EpipolarGeometry::EpipolarGeometry(const cv::Mat& K) : K_(K.clone()) {}

std::tuple<cv::Mat, cv::Mat, cv::Mat> EpipolarGeometry::EstimatePose(
    const std::vector<cv::Point2f>& kpts1, 
    const std::vector<cv::Point2f>& kpts2) {
    
    cv::Mat R, t, inlier_mask;
    
    // A minimum of 5 points is mathematically required.
    if (kpts1.size() < 5) {
        return {R, t, inlier_mask};
    }

    // The nuclear upgrade: USAC_MAGSAC for extreme outlier rejection
    cv::Mat E = cv::findEssentialMat(
        kpts1, kpts2, 
        K_, 
        cv::USAC_MAGSAC, 
        0.9999, // Extremely high confidence
        1.0,    // Epipolar threshold
        inlier_mask
    );

    // Validate the resulting Essential matrix
    if (E.empty() || E.rows != 3 || E.cols != 3) {
        return {cv::Mat(), cv::Mat(), cv::Mat()};
    }

    // Recover the relative rotation and translation
    cv::recoverPose(E, kpts1, kpts2, K_, R, t, inlier_mask);
    
    return {R, t, inlier_mask};
}

} // namespace geometry
} // namespace deepvo