#include "deepvo/visualizer.h"

#include <iostream>

namespace deepvo {
namespace visualization {

Visualizer2D::Visualizer2D(const std::string& window_name)
    : window_name_(window_name) {
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
}

Visualizer2D::~Visualizer2D() {
    cv::destroyWindow(window_name_);
}

bool Visualizer2D::ShowFrame(cv::Mat& frame,
                             const std::vector<cv::Point2f>& tracked_points,
                             const cv::Mat& position,
                             const std::string& status_text) {
    for (const auto& pt : tracked_points) {
        cv::circle(frame, pt, 3, cv::Scalar(0, 255, 0), cv::FILLED);
    }

    if (!position.empty() && position.rows == 3 && position.cols == 1) {
        const std::string pos_str = cv::format(
            "Pos: [%.2f, %.2f, %.2f]",
            position.at<double>(0), position.at<double>(1), position.at<double>(2));
        cv::putText(frame, pos_str, cv::Point(20, 36),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }

    cv::putText(frame, status_text, cv::Point(20, 70),
                cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 200, 0), 2);

    cv::imshow(window_name_, frame);

    if (cv::waitKey(30) == 27) {
        std::cout << "[Info] User interrupted the tracking." << std::endl;
        return false;
    }

    return true;
}

}  // namespace visualization
}  // namespace deepvo
