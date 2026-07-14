// =============================================================================
// KeyFrame implementation
// =============================================================================

#include "litevo/core/keyframe.h"

#include <algorithm>

#include "litevo/core/map.h"
#include "litevo/core/map_point.h"

namespace litevo {

std::mutex KeyFrame::global_mutex_;

KeyFrame::KeyFrame(const Frame& frame) : Frame(frame) {
    SetKeyFrame(true);
}

KeyFrame::KeyFrame(const cv::Mat& image, double timestamp, features::OrbExtractor& extractor,
                   const Camera& camera)
    : Frame(image, timestamp, extractor, camera) {
    SetKeyFrame(true);
}

// ── Covisibility Graph ───────────────────────────────────────────────────────

void KeyFrame::AddConnection(KeyFrame* kf, int weight) {
    std::lock_guard<std::mutex> lock(global_mutex_);
    if (kf == this)
        return;
    connected_keyframes_[kf] = weight;
}

void KeyFrame::EraseConnection(KeyFrame* kf) {
    std::lock_guard<std::mutex> lock(global_mutex_);
    connected_keyframes_.erase(kf);
}

void KeyFrame::UpdateConnections(const std::vector<std::shared_ptr<KeyFrame>>& all_kfs) {
    // For each map point this KF observes, find other KFs that also observe it
    std::map<KeyFrame*, int> weight_map;

    for (const auto& other_kf : all_kfs) {
        if (other_kf.get() == this || other_kf->IsBad())
            continue;

        int shared_count = 0;
        // Count shared map points
        for (int i = 0; i < NumKeyPoints(); ++i) {
            MapPointId mp_id = MapPointIdAt(i);
            if (mp_id.id == 0)
                continue;

            // Check if the other KF also observes this map point
            for (int j = 0; j < other_kf->NumKeyPoints(); ++j) {
                if (other_kf->MapPointIdAt(j) == mp_id) {
                    shared_count++;
                    break;
                }
            }
        }

        if (shared_count > 0) {
            weight_map[other_kf.get()] = shared_count;
        }
    }

    // Update connections (sorted by key pointer in std::map)
    {
        std::lock_guard<std::mutex> lock(global_mutex_);
        connected_keyframes_.clear();
        for (const auto& [kf_ptr, weight] : weight_map) {
            connected_keyframes_[kf_ptr] = weight;
        }
    }
}

std::vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(int N) const {
    std::lock_guard<std::mutex> lock(global_mutex_);

    // Sort by weight descending
    std::vector<std::pair<KeyFrame*, int>> sorted(connected_keyframes_.begin(),
                                                  connected_keyframes_.end());
    std::sort(sorted.begin(), sorted.end(), CompareByWeight);

    std::vector<KeyFrame*> result;
    int count = std::min(N, static_cast<int>(sorted.size()));
    result.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        result.push_back(sorted[static_cast<size_t>(i)].first);
    }
    return result;
}

std::vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(int min_weight) const {
    std::lock_guard<std::mutex> lock(global_mutex_);

    std::vector<KeyFrame*> result;
    for (const auto& [kf, weight] : connected_keyframes_) {
        if (weight >= min_weight) {
            result.push_back(kf);
        }
    }
    return result;
}

int KeyFrame::GetWeight(KeyFrame* kf) const {
    std::lock_guard<std::mutex> lock(global_mutex_);
    auto it = connected_keyframes_.find(kf);
    if (it != connected_keyframes_.end()) {
        return it->second;
    }
    return 0;
}

// ── Spanning Tree ────────────────────────────────────────────────────────────

void KeyFrame::SetParent(KeyFrame* parent) {
    std::lock_guard<std::mutex> lock(global_mutex_);
    parent_ = parent;
}

void KeyFrame::AddChild(KeyFrame* child) {
    std::lock_guard<std::mutex> lock(global_mutex_);
    children_.insert(child);
}

void KeyFrame::EraseChild(KeyFrame* child) {
    std::lock_guard<std::mutex> lock(global_mutex_);
    children_.erase(child);
}

bool KeyFrame::HasChild(KeyFrame* kf) const {
    std::lock_guard<std::mutex> lock(global_mutex_);
    return children_.contains(kf);
}

// ── Status ───────────────────────────────────────────────────────────────────

void KeyFrame::SetBad(bool flag) {
    std::lock_guard<std::mutex> lock(global_mutex_);
    is_bad_ = flag;
}

// ── MapPoint helpers ─────────────────────────────────────────────────────────

std::vector<std::shared_ptr<MapPoint>> KeyFrame::GetMapPointMatches(Map& map) const {
    std::vector<std::shared_ptr<MapPoint>> result;
    for (int i = 0; i < NumKeyPoints(); ++i) {
        MapPointId mp_id = MapPointIdAt(i);
        if (mp_id.id != 0) {
            auto mp = map.GetMapPoint(mp_id);
            if (mp && !mp->IsBad()) {
                result.push_back(mp);
            }
        }
    }
    return result;
}

// ── Private helpers ──────────────────────────────────────────────────────────

bool KeyFrame::CompareByWeight(const std::pair<KeyFrame*, int>& a,
                               const std::pair<KeyFrame*, int>& b) {
    return a.second > b.second;  // Descending
}

}  // namespace litevo
