// Copyright 2026 Maliketh. All rights reserved.
//
// Defines the data structures and classes for the backend optimization pipeline.
// This includes the sliding window manager used for Local Bundle Adjustment (BA)
// to smooth the trajectory and refine 3D map points.

#pragma once

#include <deque>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace deepvo {
namespace backend {

// Represents a single camera frame within the backend optimization window.
// It stores the estimated global pose and the observed 2D-3D point correspondences.
struct Frame {
    int frame_id;

    // The global pose of the camera in the world coordinate system (T_c_w).
    // Represented as an SE(3) transformation matrix using Eigen for high-performance
    // backend optimization operations.
    Eigen::Matrix4d T_c_w;

    // The 2D keypoints successfully tracked in this frame by the frontend.
    std::vector<cv::Point2f> observed_keypoints;

    // The corresponding 3D spatial points obtained via triangulation.
    // These points are optimized jointly with the camera poses during Bundle Adjustment.
    std::vector<Eigen::Vector3d> map_points_3d;
};

// Manages a sliding window of recent camera frames and performs Local Bundle Adjustment.
// By jointly optimizing the poses and 3D map points within this window, it effectively
// mitigates local scale drift and tracking jitter.
class SlidingWindowOptimizer {
 public:
    // Constructs the optimizer with a specified maximum window size.
    //
    // @param window_size The maximum number of frames to keep in the optimization window.
    explicit SlidingWindowOptimizer(int window_size = 10);

    // Incorporates a newly tracked frame into the sliding window.
    // Automatically marginalizes (removes) the oldest frame if the window exceeds
    // its maximum capacity.
    //
    // @param id The unique identifier for the current frame.
    // @param cv_R The 3x3 rotation matrix estimated by the frontend (CV_64F).
    // @param cv_t The 3x1 translation vector estimated by the frontend (CV_64F).
    // @param kpts The 2D keypoints actively tracked in this frame.
    void AddFrame(int id, const cv::Mat& cv_R, const cv::Mat& cv_t,
                  const std::vector<cv::Point2f>& kpts);

    // Executes the Local Bundle Adjustment process over the current sliding window.
    //
    // @return The optimized 4x4 SE(3) pose matrix of the most recent frame in the window.
    Eigen::Matrix4d Optimize();

 private:
    // Converts OpenCV rotation and translation matrices into a unified Eigen 4x4 matrix.
    //
    // @param cv_R The 3x3 OpenCV rotation matrix.
    // @param cv_t The 3x1 OpenCV translation vector.
    // @return An Eigen::Matrix4d representing the SE(3) transformation.
    Eigen::Matrix4d ConvertToEigen(const cv::Mat& cv_R, const cv::Mat& cv_t);

    int window_size_;
    std::deque<Frame> window_;  // The core container for the sliding window frames.
};

}  // namespace backend
}  // namespace deepvo