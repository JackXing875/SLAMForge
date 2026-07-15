// =============================================================================
// LoopClosing — orchestrator thread for loop detection and global optimization
// =============================================================================
// The third background thread in the ORB-SLAM3-style architecture.
//
// Lifecycle:
//   1. Created by the application (main thread)
//   2. Started → enters Run() loop, waiting for new keyframes
//   3. For each new KF:
//      a. LoopDetector queries for candidates
//      b. LoopVerifier checks geometrically
//      c. LoopCorrector applies Sim(3) and fuses points
//      d. PoseGraphOptimizer runs global pose graph optimization
//      e. GlobalBundleAdjuster is launched in a separate thread
//   4. Stopped → cleanly exits the Run() loop

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "litevo/core/config.h"
#include "litevo/loop_closing/corrector.h"
#include "litevo/loop_closing/detector.h"
#include "litevo/loop_closing/global_ba.h"
#include "litevo/loop_closing/pose_graph.h"
#include "litevo/loop_closing/verifier.h"
#include "litevo/loop_closing/vocabulary.h"

namespace litevo {

class Camera;
class KeyFrame;
class Map;

namespace features {
class OrbExtractor;
}

namespace loop_closing {

/// @brief Main loop closing thread — orchestrates detection, verification,
///        correction, and global optimization.
///
/// Follows the same thread pattern as LocalMapper:
///   - Producer-consumer: Tracker pushes KFs to queue, LoopClosing processes
///   - Start/Stop/RequestStop lifecycle
///   - Atomic flags for state queries
class LoopClosing {
public:
    using KfQueue = std::queue<std::shared_ptr<KeyFrame>>;

    /// @param map        The shared SLAM map.
    /// @param camera     Camera model (for verification / BA).
    /// @param config     Loop closing configuration.
    LoopClosing(Map& map, const Camera& camera, const LoopClosingConfig& config);

    ~LoopClosing();

    // Non-copyable (owns thread)
    LoopClosing(const LoopClosing&) = delete;
    LoopClosing& operator=(const LoopClosing&) = delete;

    // ── Thread control ──────────────────────────────────────────────────────

    /// @brief Start the loop closing thread.
    void Start();

    /// @brief Stop and join the thread.
    void Stop();

    /// @brief Request stop (non-blocking).
    void RequestStop();

    bool IsRunning() const { return running_; }
    bool IsFinished() const { return is_finished_; }

    // ── Input ───────────────────────────────────────────────────────────────

    /// @brief Insert a new keyframe for loop detection. Thread-safe.
    /// Should be called from the Tracking thread after a KF is created.
    void InsertKeyFrame(std::shared_ptr<KeyFrame> kf);

    /// @brief Approximate queue size.
    int QueueSize() const;

    // ── Vocabulary ──────────────────────────────────────────────────────────

    /// @brief Load BoW vocabulary from file (must be called before Start).
    bool LoadVocabulary(const std::string& path);

    /// @brief Whether a BoW vocabulary was loaded successfully.
    bool HasVocabulary() const { return detector_.GetVocabulary().IsLoaded(); }

    /// @brief Effective settings for the detector and pose-graph optimizer.
    /// Useful for diagnostics and for verifying loaded configuration.
    const LoopDetectorConfig& DetectorSettings() const { return detector_.Config(); }
    const PoseGraphConfig& PoseGraphSettings() const { return pose_graph_.Config(); }

    // ── State ───────────────────────────────────────────────────────────────

    /// @brief Whether a loop was detected and corrected in the last processing.
    bool LoopDetected() const { return loop_detected_; }

    /// @brief Number of loops closed since start.
    int NumLoopsClosed() const { return num_loops_closed_; }

private:
    // ── Main loop ───────────────────────────────────────────────────────────

    void Run();

    // ── Components ──────────────────────────────────────────────────────────

    Map& map_;
    const Camera& camera_;
    LoopClosingConfig config_;

    LoopDetector detector_;
    LoopVerifier verifier_;
    LoopCorrector corrector_;
    PoseGraphOptimizer pose_graph_;
    GlobalBundleAdjuster global_ba_;

    // ── Threading ───────────────────────────────────────────────────────────

    std::thread thread_;
    mutable std::mutex lifecycle_mutex_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    KfQueue kf_queue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> is_finished_{false};

    // ── Statistics ──────────────────────────────────────────────────────────

    std::atomic<bool> loop_detected_{false};
    std::atomic<int> num_loops_closed_{0};
};

}  // namespace loop_closing
}  // namespace litevo
