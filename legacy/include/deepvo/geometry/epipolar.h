#pragma once

#include <vector>
#include <tuple>
#include <opencv2/opencv.hpp>

namespace deepvo {
namespace geometry {

/**
 * @class EpipolarGeometry
 * @brief Handles essential matrix estimation and pose recovery.
 */
class EpipolarGeometry {
public:
    /**
     * @brief Constructs the EpipolarGeometry solver with camera intrinsics.
     * @param K The 3x3 camera intrinsic matrix.
     */
    explicit EpipolarGeometry(const cv::Mat& K);

    /**
     * @brief Estimates camera motion (R, t) between two frames.
     * * @param kpts1 Keypoints from the first frame.
     * @param kpts2 Matched keypoints from the second frame.
     * @return A tuple containing {Rotation (3x3), Translation (3x1), Inlier Mask (Nx1)}.
     * Returns empty cv::Mat objects if estimation fails.
     */
    std::tuple<cv::Mat, cv::Mat, cv::Mat> EstimatePose(
        const std::vector<cv::Point2f>& kpts1, 
        const std::vector<cv::Point2f>& kpts2);

private:
    cv::Mat K_; // Camera intrinsic matrix
};

} // namespace geometry
} // namespace deepvo