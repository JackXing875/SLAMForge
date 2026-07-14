// =============================================================================
// MapPoint implementation
// =============================================================================

#include "litevo/core/map_point.h"

#include <cmath>

namespace litevo {

uint64_t MapPoint::next_id_ = 1;  // Start at 1 so id=0 means invalid

MapPoint::MapPoint(const Vec3& position, FrameId reference_frame)
    : id_{next_id_++}, position_(position), reference_frame_(reference_frame) {
    normal_ = Vec3(0, 0, 1);
}

void MapPoint::SetDescriptor(const cv::Mat& desc) {
    desc.copyTo(descriptor_);
}

void MapPoint::AddObservation(FrameId frame_id) {
    observed_by_.insert(frame_id);
    observations_ = static_cast<int>(observed_by_.size());
}

void MapPoint::EraseObservation(FrameId frame_id) {
    observed_by_.erase(frame_id);
    observations_ = static_cast<int>(observed_by_.size());
}

void MapPoint::UpdateNormal(const Vec3& camera_center) {
    // Average of all viewing directions
    Vec3 view_dir = (camera_center - position_).normalized();
    // Running average: blend old normal with new viewing direction
    if (observations_ <= 1) {
        normal_ = view_dir;
    } else {
        double weight = 1.0 / observations_;
        normal_ = (normal_ * (1.0 - weight) + view_dir * weight).normalized();
    }
}

float MapPoint::GetFoundRatio() const {
    if (visible_count_ == 0)
        return -1.0f;
    return static_cast<float>(found_count_) / static_cast<float>(visible_count_);
}

bool MapPoint::IsEraseReady(int min_observations) const {
    // Ready to erase if:
    // 1. Too few observations AND
    // 2. Either never found (found_count_ == 0) or found ratio < 25%
    if (observations_ >= min_observations)
        return false;

    // For recently created points (first 2 frames), don't erase yet
    if (frames_since_creation_ <= 2)
        return false;

    if (found_count_ == 0)
        return true;

    if (visible_count_ > 0) {
        float ratio = GetFoundRatio();
        if (ratio >= 0.0f && ratio < 0.25f)
            return true;
    }

    return false;
}

int MapPoint::PredictScale(float current_dist, int num_levels, float scale_factor,
                           float log_scale_factor) const {
    // For the first observation, predict level 0
    if (max_distance_ <= 0.0f) {
        return 0;
    }

    // Ratio of max observable distance to current distance
    float ratio = max_distance_ / current_dist;

    // Convert ratio to pyramid level
    int level = static_cast<int>(std::ceil(std::log(ratio) / log_scale_factor));
    if (level < 0)
        level = 0;
    if (level >= num_levels)
        level = num_levels - 1;

    return level;
}

void MapPoint::Replace(MapPoint* other) {
    if (!other || other == this)
        return;

    int my_original_obs = observations_;
    int other_original_obs = other->observations_;

    // Transfer observations from other to this
    for (const auto& obs : other->observed_by_) {
        observed_by_.insert(obs);
    }
    observations_ = static_cast<int>(observed_by_.size());

    // If other has more observations, adopt its position (more trusted)
    if (other_original_obs > my_original_obs) {
        position_ = other->position_;
    }

    // Use the better descriptor (prefer the one with more observations)
    if (other->Descriptor().rows > 0 && other_original_obs >= my_original_obs) {
        other->Descriptor().copyTo(descriptor_);
    }

    // Mark other as having minimal observations (will be culled)
    other->observations_ = 0;
    other->observed_by_.clear();
    other->found_count_ = 0;
    other->visible_count_ = 0;
}

}  // namespace litevo
