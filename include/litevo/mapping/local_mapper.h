// =============================================================================
// LocalMapper — background thread for map maintenance and optimization
// =============================================================================

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "litevo/core/config.h"
#include "litevo/core/types.h"

namespace litevo {

class Camera;
class KeyFrame;
class Map;
class MapPoint;

namespace features {
class OrbExtractor;
}

namespace mapping {

/// @brief Background thread that processes new keyframes as they arrive
///        from the Tracker.
///
/// For each new keyframe, the LocalMapper:
///   1. Updates the covisibility graph
///   2. Creates new map points via triangulation with covisible KFs
///   3. Culls bad map points
///   4. Runs local bundle adjustment (Ceres)
///   5. Culls redundant keyframes
class LocalMapper {
public:
    using KfQueue = std::queue<std::shared_ptr<KeyFrame>>;

    /// @param map        The shared SLAM map.
    /// @param camera     Camera model (read-only).
    /// @param config     Mapping configuration.
    /// @param extractor  ORB extractor for feature extraction on new frames.
    LocalMapper(Map& map, const Camera& camera,
                const MappingConfig& config,
                features::OrbExtractor& extractor);

    ~LocalMapper();

    // ── Thread control ──────────────────────────────────────────────────

    /// @brief Start the mapping thread.
    void Start();

    /// @brief Stop and join the mapping thread.
    void Stop();

    /// @brief Request the thread to stop (non-blocking).
    void RequestStop();

    bool IsRunning() const { return running_; }
    bool IsFinished() const { return is_finished_; }

    // ── Interface for Tracker ───────────────────────────────────────────

    /// @brief Insert a new keyframe for processing. Thread-safe.
    void InsertKeyFrame(std::shared_ptr<KeyFrame> kf);

    /// @brief Approximate number of keyframes in queue.
    int QueueSize() const;

    /// @brief Pause/resume accepting new keyframes.
    void SetAcceptKeyFrames(bool accept) { accept_keyframes_ = accept; }
    bool IsAcceptingKeyFrames() const { return accept_keyframes_; }

private:
    // ── Main loop ───────────────────────────────────────────────────────

    void Run();

    // ── Processing steps ────────────────────────────────────────────────

    /// @brief Associate MapPoints, update covisibility graph, add to spanning tree.
    void ProcessNewKeyFrame();

    /// @brief Triangulate new MapPoints with covisible keyframes.
    void CreateNewMapPoints();

    /// @brief Remove map points that fail quality checks.
    void CullMapPoints();

    /// @brief Run Ceres local bundle adjustment.
    void LocalBundleAdjustment();

    /// @brief Remove keyframes that are redundant (>90% points seen by others).
    void CullKeyFrames();

    // ── Helpers ─────────────────────────────────────────────────────────

    /// @brief Rebuild the covisibility connections for a keyframe.
    void UpdateCovisibilityGraph(std::shared_ptr<KeyFrame> kf);

    /// @brief Triangulate new points between two keyframes.
    /// @return Number of new map points created.
    int TriangulateWithKf(
        KeyFrame* kf1, KeyFrame* kf2,
        std::vector<std::shared_ptr<MapPoint>>& new_mps);

    /// @brief Search for matching features between two KFs for triangulation.
    int SearchForTriangulation(
        KeyFrame* kf1, KeyFrame* kf2,
        std::vector<std::pair<int, int>>& matches) const;

    // ── Members ─────────────────────────────────────────────────────────

    Map& map_;
    const Camera& camera_;
    MappingConfig config_;
    features::OrbExtractor& extractor_;

    // Threading
    std::thread thread_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    KfQueue new_keyframes_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> is_finished_{false};
    std::atomic<bool> accept_keyframes_{true};

    // Current processing state
    std::shared_ptr<KeyFrame> current_kf_;
};

}  // namespace mapping
}  // namespace litevo
