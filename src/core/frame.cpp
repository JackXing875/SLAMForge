// =============================================================================
// Frame implementation
// =============================================================================

#include "litevo/core/frame.h"

#include <opencv2/imgproc.hpp>

#include "litevo/features/orb_extractor.h"

namespace litevo {

uint64_t Frame::next_id_ = 0;

Frame::Frame(const cv::Mat& image, double timestamp,
             features::OrbExtractor& extractor, const Camera& camera)
    : timestamp_(timestamp)
    , camera_(camera)
{
    // Assign unique ID
    id_ = FrameId{next_id_++};

    // Ensure grayscale
    if (image.channels() == 3) {
        cv::cvtColor(image, image_, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, image_, cv::COLOR_BGRA2GRAY);
    } else {
        image_ = image.clone();
    }

    // Extract features
    ExtractFeatures(extractor);

    // Undistort keypoints
    UndistortKeyPoints();

    // Build spatial grid
    AssignFeaturesToGrid();

    // Initialize map point associations to null
    map_point_ids_.assign(keypoints_.size(), MapPointId{0});
}

void Frame::ExtractFeatures(features::OrbExtractor& extractor) {
    extractor.ExtractUniform(image_, keypoints_, descriptors_);
}

void Frame::UndistortKeyPoints() {
    keypoints_undistorted_.reserve(keypoints_.size());

    if (!camera_.has_distortion()) {
        // No distortion – just copy
        keypoints_undistorted_ = keypoints_;
        return;
    }

    for (const auto& kp : keypoints_) {
        // Convert to normalized coordinates
        double nx = (kp.pt.x - camera_.cx()) / camera_.fx();
        double ny = (kp.pt.y - camera_.cy()) / camera_.fy();

        // Undistort
        Vec2 norm_undist = camera_.Undistort(Vec2(nx, ny));

        // Convert back to pixel coordinates
        cv::KeyPoint kp_u = kp;
        kp_u.pt.x = static_cast<float>(norm_undist.x() * camera_.fx() + camera_.cx());
        kp_u.pt.y = static_cast<float>(norm_undist.y() * camera_.fy() + camera_.cy());

        keypoints_undistorted_.push_back(kp_u);
    }
}

void Frame::AssignFeaturesToGrid() {
    grid_ = FeatureGrid(camera_.width(), camera_.height());
    grid_.Assign(keypoints_undistorted_);
}

Vec3 Frame::CameraCenter() const {
    // For Tcw = [R | t], camera center in world is: C = -R^T * t
    return -Tcw_.rotation().transpose() * Tcw_.translation();
}

MapPointId Frame::MapPointIdAt(int idx) const {
    if (idx < 0 || idx >= static_cast<int>(map_point_ids_.size())) {
        return MapPointId{0};
    }
    return map_point_ids_[static_cast<size_t>(idx)];
}

void Frame::SetMapPointId(int idx, MapPointId mp_id) {
    if (idx >= 0 && idx < static_cast<int>(map_point_ids_.size())) {
        map_point_ids_[static_cast<size_t>(idx)] = mp_id;
    }
}

int Frame::NumMapPoints() const {
    int count = 0;
    for (const auto& mp_id : map_point_ids_) {
        if (mp_id.id != 0) {
            ++count;
        }
    }
    return count;
}

std::vector<int> Frame::GetMapPointIndices() const {
    std::vector<int> indices;
    indices.reserve(map_point_ids_.size());
    for (size_t i = 0; i < map_point_ids_.size(); ++i) {
        if (map_point_ids_[i].id != 0) {
            indices.push_back(static_cast<int>(i));
        }
    }
    return indices;
}

std::vector<int> Frame::GetFeaturesInArea(
    float x, float y, float radius, int min_level, int max_level) const {
    return grid_.GetCandidates(x, y, radius, min_level, max_level);
}

}  // namespace litevo
