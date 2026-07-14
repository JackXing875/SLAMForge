// =============================================================================
// Map — thread-safe container for all SLAM map elements
// =============================================================================

#pragma once

#include <memory>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include "litevo/core/types.h"

namespace litevo {

class Frame;
class KeyFrame;
class MapPoint;

/// @brief Thread-safe container holding all MapPoints and KeyFrames.
///
/// Uses std::shared_mutex: shared_lock for reads, unique_lock for writes.
/// The Tracker (main thread) and LocalMapper (background thread) access
/// the map concurrently.
class Map {
public:
    Map() = default;

    // ── MapPoint operations ────────────────────────────────────────────────

    /// @brief Create a new MapPoint and add it to the map.
    std::shared_ptr<MapPoint> AddMapPoint(const Vec3& position, FrameId ref_frame);

    /// @brief Insert a pre-existing MapPoint into the map.
    /// Used when the MapPoint already has descriptors, observations, etc.
    void InsertMapPoint(std::shared_ptr<MapPoint> mp);

    /// @brief Remove a MapPoint by ID.
    void EraseMapPoint(MapPointId id);

    /// @brief Get a MapPoint by ID, or nullptr if not found.
    std::shared_ptr<MapPoint> GetMapPoint(MapPointId id) const;

    /// @brief Number of MapPoints.
    size_t MapPointCount() const;

    /// @brief Get all MapPoints.
    std::vector<std::shared_ptr<MapPoint>> GetAllMapPoints() const;

    // ── KeyFrame operations ────────────────────────────────────────────────

    /// @brief Add a KeyFrame to the map. Thread-safe.
    void AddKeyFrame(std::shared_ptr<KeyFrame> frame);

    /// @brief Get a keyframe by FrameId, or nullptr if not found.
    std::shared_ptr<KeyFrame> GetKeyFrame(FrameId id) const;

    /// @brief Number of keyframes.
    size_t KeyFrameCount() const;

    /// @brief Get all keyframes.
    std::vector<std::shared_ptr<KeyFrame>> GetAllKeyFrames() const;

    /// @brief Get the most recent N keyframes (for relocalization).
    std::vector<std::shared_ptr<KeyFrame>> GetRecentKeyFrames(int n) const;

    /// @brief Reference (last inserted) keyframe.
    std::shared_ptr<KeyFrame> ReferenceKeyFrame() const;
    void SetReferenceKeyFrame(std::shared_ptr<KeyFrame> kf);

    // ── Maintenance ────────────────────────────────────────────────────────

    /// @brief Remove all MapPoints and KeyFrames.
    void Clear();

private:
    mutable std::shared_mutex map_mutex_;

    std::unordered_map<MapPointId, std::shared_ptr<MapPoint>, MapPointIdHash> map_points_;
    std::vector<std::shared_ptr<KeyFrame>> keyframes_;
    std::shared_ptr<KeyFrame> reference_kf_;
};

}  // namespace litevo
