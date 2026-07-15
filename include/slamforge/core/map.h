// =============================================================================
// Map — thread-safe container for all SLAM map elements
// =============================================================================

#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include "slamforge/core/types.h"

namespace slamforge {

class Frame;
class KeyFrame;
class MapPoint;

/// @brief Thread-safe container holding all MapPoints and KeyFrames.
///
/// Uses a graph transaction lock for coherent keyframe/landmark updates plus
/// a std::shared_mutex for the owning collections.  The Tracker, LocalMapper,
/// LoopClosing, and GlobalBA workers access the map concurrently.
class Map {
public:
    /// @brief Exclusive guard for a coherent map-graph transaction.
    ///
    /// The container mutex protects only the keyframe/map-point collections;
    /// poses, feature associations, and graph edges live in the objects those
    /// collections own.  Tracker, LocalMapper, LoopClosing, and GlobalBA take
    /// this guard around their graph-level operations so loop correction and
    /// BA cannot race on those object fields.  It is recursive to allow the
    /// guarded workflows to call the regular Map accessors safely.
    using GraphLock = std::unique_lock<std::recursive_mutex>;

    Map() = default;

    [[nodiscard]] GraphLock AcquireGraphLock() const { return GraphLock(graph_mutex_); }

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
    mutable std::recursive_mutex graph_mutex_;
    mutable std::shared_mutex map_mutex_;

    std::unordered_map<MapPointId, std::shared_ptr<MapPoint>, MapPointIdHash> map_points_;
    std::vector<std::shared_ptr<KeyFrame>> keyframes_;
    std::shared_ptr<KeyFrame> reference_kf_;
};

}  // namespace slamforge
