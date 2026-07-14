// =============================================================================
// Map implementation
// =============================================================================

#include "litevo/core/map.h"

#include <algorithm>

#include "litevo/core/keyframe.h"
#include "litevo/core/map_point.h"

namespace litevo {

std::shared_ptr<MapPoint> Map::AddMapPoint(const Vec3& position, FrameId ref_frame) {
    std::unique_lock lock(map_mutex_);
    auto mp = std::make_shared<MapPoint>(position, ref_frame);
    map_points_[mp->Id()] = mp;
    return mp;
}

void Map::InsertMapPoint(std::shared_ptr<MapPoint> mp) {
    if (!mp) return;
    std::unique_lock lock(map_mutex_);
    map_points_[mp->Id()] = mp;
}

void Map::EraseMapPoint(MapPointId id) {
    std::unique_lock lock(map_mutex_);
    map_points_.erase(id);
}

std::shared_ptr<MapPoint> Map::GetMapPoint(MapPointId id) const {
    std::shared_lock lock(map_mutex_);
    auto it = map_points_.find(id);
    if (it != map_points_.end()) {
        return it->second;
    }
    return nullptr;
}

size_t Map::MapPointCount() const {
    std::shared_lock lock(map_mutex_);
    return map_points_.size();
}

std::vector<std::shared_ptr<MapPoint>> Map::GetAllMapPoints() const {
    std::shared_lock lock(map_mutex_);
    std::vector<std::shared_ptr<MapPoint>> result;
    result.reserve(map_points_.size());
    for (const auto& [id, mp] : map_points_) {
        result.push_back(mp);
    }
    return result;
}

void Map::AddKeyFrame(std::shared_ptr<KeyFrame> frame) {
    std::unique_lock lock(map_mutex_);
    frame->SetKeyFrame(true);
    keyframes_.push_back(frame);
    reference_kf_ = frame;
}

std::shared_ptr<KeyFrame> Map::GetKeyFrame(FrameId id) const {
    std::shared_lock lock(map_mutex_);
    for (const auto& kf : keyframes_) {
        if (kf->Id() == id) {
            return kf;
        }
    }
    return nullptr;
}

size_t Map::KeyFrameCount() const {
    std::shared_lock lock(map_mutex_);
    return keyframes_.size();
}

std::vector<std::shared_ptr<KeyFrame>> Map::GetAllKeyFrames() const {
    std::shared_lock lock(map_mutex_);
    return keyframes_;
}

std::vector<std::shared_ptr<KeyFrame>> Map::GetRecentKeyFrames(int n) const {
    std::shared_lock lock(map_mutex_);
    std::vector<std::shared_ptr<KeyFrame>> result;
    int start = std::max(0, static_cast<int>(keyframes_.size()) - n);
    for (int i = start; i < static_cast<int>(keyframes_.size()); ++i) {
        result.push_back(keyframes_[static_cast<size_t>(i)]);
    }
    return result;
}

std::shared_ptr<KeyFrame> Map::ReferenceKeyFrame() const {
    std::shared_lock lock(map_mutex_);
    return reference_kf_;
}

void Map::SetReferenceKeyFrame(std::shared_ptr<KeyFrame> kf) {
    std::unique_lock lock(map_mutex_);
    reference_kf_ = kf;
}

void Map::Clear() {
    std::unique_lock lock(map_mutex_);
    map_points_.clear();
    keyframes_.clear();
    reference_kf_.reset();
}

}  // namespace litevo
