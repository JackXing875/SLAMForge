/**
 * @file main.cpp
 * @brief Main entry point for the C++ DeepVO pipeline.
 */

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "deepvo/tracker.h"
#include "deepvo/visualizer.h"

namespace {

cv::Mat BuildIntrinsics(double width, double height, int argc, char* argv[]) {
    if (argc == 7) {
        return (cv::Mat_<double>(3, 3) <<
            std::stod(argv[3]), 0.0, std::stod(argv[5]),
            0.0, std::stod(argv[4]), std::stod(argv[6]),
            0.0, 0.0, 1.0);
    }

    const double cx = width * 0.5;
    const double cy = height * 0.5;
    const double focal = 0.9 * std::max(width, height);
    return (cv::Mat_<double>(3, 3) <<
        focal, 0.0, cx,
        0.0, focal, cy,
        0.0, 0.0, 1.0);
}

}  // namespace

int main(int argc, char* argv[]) {
    std::cout << "  DeepVO Sparse Monocular Engine Initialized   " << std::endl;

    if (argc != 3 && argc != 7) {
        std::cerr << "[Error] Invalid arguments." << std::endl;
        std::cerr << "Usage: ./deepvo_app <input_video_path> <output_csv_path> [fx fy cx cy]" << std::endl;
        return -1;
    }

    const std::string video_source = argv[1];
    const std::string output_csv = argv[2];

    cv::VideoCapture cap;
    if (video_source == "0") {
        cap.open(0);
    } else {
        cap.open(video_source);
    }

    if (!cap.isOpened()) {
        std::cerr << "[Error] Cannot open video stream: " << video_source << std::endl;
        return -1;
    }

    const double frame_width = std::max(1.0, cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const double frame_height = std::max(1.0, cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const cv::Mat K = BuildIntrinsics(frame_width, frame_height, argc, argv);
    std::cout << "[Info] Using intrinsics:\n" << K << std::endl;

    deepvo::tracker::VisualOdometryTracker tracker(K);
    deepvo::visualization::Visualizer2D viewer("DeepVO - Sparse VO");

    std::ofstream traj_file(output_csv);
    if (!traj_file.is_open()) {
        std::cerr << "[Error] Could not open trajectory CSV for writing: " << output_csv << std::endl;
        return -1;
    }
    traj_file << "x,y,z\n";

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cout << "[Info] End of video stream." << std::endl;
            break;
        }

        if (!tracker.ProcessFrame(frame)) {
            continue;
        }

        const cv::Mat position = tracker.GetCurrentPosition();
        if (tracker.HasActivePose() &&
            !position.empty() && position.rows == 3 && position.cols == 1) {
            traj_file << position.at<double>(0) << ","
                      << position.at<double>(1) << ","
                      << position.at<double>(2) << "\n";
            traj_file.flush();
        }

        const std::vector<cv::Point2f> tracked_points = tracker.GetTrackedPoints();
        if (!viewer.ShowFrame(frame, tracked_points, position, tracker.GetStatusText())) {
            break;
        }
    }

    cap.release();
    traj_file.close();
    std::cout << "[Info] Shutting down gracefully." << std::endl;
    return 0;
}
