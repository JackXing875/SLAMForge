// =============================================================================
// GlobalBundleAdjuster — full BA in a separate thread after loop closure
// =============================================================================
// Runs global bundle adjustment over all keyframes and map points in a
// dedicated thread to avoid blocking the tracking thread.

#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include "litevo/core/types.h"

namespace litevo {

class Camera;
class Map;

namespace loop_closing {

/// @brief Configuration for GlobalBundleAdjuster.
struct GlobalBAConfig {
    int max_iterations = 20;
    double huber_delta = 5.0;
    bool use_huber = true;
};

/// @brief Full bundle adjustment in a background thread.
///
/// Creates its own thread for the optimization. The calling thread
/// can check IsFinished() to know when the optimization is done.
class GlobalBundleAdjuster {
public:
    GlobalBundleAdjuster() = default;
    ~GlobalBundleAdjuster();

    // Non-copyable (owns thread)
    GlobalBundleAdjuster(const GlobalBundleAdjuster&) = delete;
    GlobalBundleAdjuster& operator=(const GlobalBundleAdjuster&) = delete;

    /// @brief Start the global BA thread.
    /// @param map  The map to optimize.
    /// @param camera  Camera model for reprojection.
    /// @param config  Solver configuration.
    void Start(Map& map, const Camera& camera, const GlobalBAConfig& config);

    /// @brief Request the thread to stop (non-blocking).
    void RequestStop();

    /// @brief Check if the thread is currently running.
    bool IsRunning() const { return running_; }

    /// @brief Check if the thread has finished.
    bool IsFinished() const { return is_finished_; }

    /// @brief Number of iterations actually performed.
    int Iterations() const { return iterations_; }

    /// @brief Final cost (reprojection error).
    double FinalCost() const { return final_cost_; }

private:
    void Run();

    Map* map_ = nullptr;
    const Camera* camera_ = nullptr;
    GlobalBAConfig config_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> is_finished_{false};
    std::atomic<int> iterations_{0};
    std::atomic<double> final_cost_{0.0};
};

}  // namespace loop_closing
}  // namespace litevo
