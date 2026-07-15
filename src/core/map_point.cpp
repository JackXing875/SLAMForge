// =============================================================================
// MapPoint implementation
// =============================================================================

#include "litevo/core/map_point.h"

#include <cmath>

namespace litevo {

std::atomic<uint64_t> MapPoint::next_id_{1};  // id=0 is the invalid sentinel

MapPoint::MapPoint(const Vec3& position, FrameId reference_frame)
    : id_{next_id_.fetch_add(1, std::memory_order_relaxed)},
      position_(position),
      reference_frame_(reference_frame) {
    normal_ = Vec3(0, 0, 1);
}

Vec3 MapPoint::Normal() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return normal_;
}

cv::Mat MapPoint::Descriptor() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return descriptor_;
}

void MapPoint::SetDescriptor(const cv::Mat& desc) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    desc.copyTo(descriptor_);
}

int MapPoint::Observations() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return observations_;
}

void MapPoint::AddObservation(FrameId frame_id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto [_, inserted] = observed_by_.insert(frame_id);
    if (!inserted) {
        return;
    }

    observations_ = static_cast<int>(observed_by_.size());

    // A keyframe observation is a verified find.  Counting it here gives
    // newly triangulated two-view points a meaningful initial quality ratio
    // instead of treating them as never observed.
    ++visible_count_;
    ++found_count_;
}

void MapPoint::EraseObservation(FrameId frame_id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    observed_by_.erase(frame_id);
    observations_ = static_cast<int>(observed_by_.size());
}

void MapPoint::UpdateNormal(const Vec3& camera_center) {
    // Average of all viewing directions
    const Vec3 position = Position();
    const Vec3 direction = camera_center - position;
    if (direction.squaredNorm() <= 1e-16) {
        return;
    }

    const Vec3 view_dir = direction.normalized();

    std::lock_guard<std::mutex> lock(data_mutex_);
    // Running average: blend old normal with new viewing direction
    if (observations_ <= 1) {
        normal_ = view_dir;
    } else {
        double weight = 1.0 / observations_;
        normal_ = (normal_ * (1.0 - weight) + view_dir * weight).normalized();
    }
}

FrameId MapPoint::ReferenceFrame() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return reference_frame_;
}

void MapPoint::SetObserved(bool val) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    is_observed_ = val;
}

bool MapPoint::IsObserved() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return is_observed_;
}

bool MapPoint::MarkObserved() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (is_observed_) {
        return false;
    }
    is_observed_ = true;
    return true;
}

void MapPoint::SetFound(bool val) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    is_found_ = val;
}

bool MapPoint::IsFound() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return is_found_;
}

int MapPoint::FoundCount() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return found_count_;
}

void MapPoint::IncreaseFound(int n) {
    if (n <= 0) {
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    found_count_ += n;
}

void MapPoint::ResetFound() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    is_found_ = false;
    is_observed_ = false;
}

void MapPoint::SetReferenceFrame(FrameId ref_frame) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    reference_frame_ = ref_frame;
}

bool MapPoint::IsBad() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return is_bad_;
}

void MapPoint::SetBad(bool bad) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    is_bad_ = bad;
}

float MapPoint::GetFoundRatio() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (visible_count_ == 0)
        return -1.0f;
    return static_cast<float>(found_count_) / static_cast<float>(visible_count_);
}

bool MapPoint::IsEraseReady(int min_observations) const {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (is_bad_) {
        return true;
    }

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
        const float ratio = static_cast<float>(found_count_) / static_cast<float>(visible_count_);
        if (ratio >= 0.0f && ratio < 0.25f)
            return true;
    }

    return false;
}

void MapPoint::IncreaseVisible(int n) {
    if (n <= 0) {
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    visible_count_ += n;
}

int MapPoint::VisibleCount() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return visible_count_;
}

int MapPoint::FramesSinceCreation() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return frames_since_creation_;
}

void MapPoint::IncrementFrame() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ++frames_since_creation_;
}

int MapPoint::PredictScale(float current_dist, int num_levels, float /*scale_factor*/,
                           float log_scale_factor) const {
    std::lock_guard<std::mutex> lock(data_mutex_);

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

    std::scoped_lock data_lock(data_mutex_, other->data_mutex_);

    int my_original_obs = observations_;
    int other_original_obs = other->observations_;

    // Transfer observations from other to this
    for (const auto& obs : other->observed_by_) {
        observed_by_.insert(obs);
    }
    observations_ = static_cast<int>(observed_by_.size());

    // If other has more observations, adopt its position (more trusted)
    if (other_original_obs > my_original_obs) {
        std::scoped_lock position_lock(pos_mutex_, other->pos_mutex_);
        position_ = other->position_;
    }

    // Use the better descriptor (prefer the one with more observations)
    if (!other->descriptor_.empty() && other_original_obs >= my_original_obs) {
        other->descriptor_.copyTo(descriptor_);
    }

    // The absorbed point must no longer be used by tracking or optimization.
    other->is_bad_ = true;
    other->observations_ = 0;
    other->observed_by_.clear();
    other->found_count_ = 0;
    other->visible_count_ = 0;
}

}  // namespace litevo
