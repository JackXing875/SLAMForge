// Implements the SlidingWindowOptimizer class for backend trajectory smoothing.

#include "deepvo/backend.h"

#include <iostream>

namespace deepvo {
namespace backend {

SlidingWindowOptimizer::SlidingWindowOptimizer(int window_size)
    : window_size_(window_size) {}

Eigen::Matrix4d SlidingWindowOptimizer::ConvertToEigen(const cv::Mat& cv_R,
                                                       const cv::Mat& cv_t) {
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_t;

    // Bridge the boundary between the OpenCV frontend and the Eigen backend.
    // This maps the underlying CV_64F memory directly to Eigen types without deep copying.
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_t, eigen_t);

    // Construct the 4x4 SE(3) transformation matrix.
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = eigen_R;
    T.block<3, 1>(0, 3) = eigen_t;

    return T;
}

void SlidingWindowOptimizer::AddFrame(int id, const cv::Mat& cv_R, const cv::Mat& cv_t,
                                      const std::vector<cv::Point2f>& kpts) {
    Frame new_frame;
    new_frame.frame_id = id;
    new_frame.T_c_w = ConvertToEigen(cv_R, cv_t);
    new_frame.observed_keypoints = kpts;

    window_.push_back(new_frame);

    // Maintain the fixed window size by evicting the oldest frame.
    // In a complete SLAM system, this step involves marginalization (e.g., Schur complement)
    // to preserve the prior geometric constraints of the evicted frame.
    if (window_.size() > window_size_) {
        window_.pop_front();
    }

    std::cout << "[Backend] Frame " << id << " added to sliding window. Current size: "
              << window_.size() << std::endl;
}

Eigen::Matrix4d SlidingWindowOptimizer::Optimize() {
    // Optimization requires at least two frames to establish stereo constraints.
    if (window_.size() < 2) {
        return window_.back().T_c_w;
    }

    std::cout << "[Backend] Triggering Local Bundle Adjustment over "
              << window_.size() << " frames..." << std::endl;

    // TODO: 1. Execute multi-view triangulation to generate or update 3D map points.
    // TODO: 2. Initialize Ceres Solver problem and construct the non-linear least squares graph.
    // TODO: 3. Compute the Jacobians and update the T_c_w for all frames in the window.

    // Temporarily return the unoptimized pose of the most recent frame
    // until the Ceres solver is fully integrated.
    return window_.back().T_c_w;
}

}  // namespace backend
}  // namespace deepvo