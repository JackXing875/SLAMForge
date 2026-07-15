// =============================================================================
// PoseGraphOptimizer — g2o-based pose graph optimization after loop closure
// =============================================================================
// Builds a pose graph from the SLAM map and optimizes all keyframe poses
// globally. Edges include:
//   - Spanning tree edges (binary edges connecting parent-child KFs)
//   - Covisibility edges (KFs sharing ≥ 100 map points)
//   - Loop closure edge (the newly detected loop constraint)

#pragma once

#include <memory>

#include "slamforge/core/types.h"

namespace slamforge {

class KeyFrame;
class Map;

namespace loop_closing {

/// @brief Configuration for PoseGraphOptimizer.
struct PoseGraphConfig {
    int max_iterations = 20;            ///< Max optimizer iterations
    int min_covisibility_weight = 100;  ///< Min shared MPs for covis edge
    bool verbose = false;
    double loop_edge_weight = 100.0;  ///< Information weight for loop edges
};

/// @brief Optimizes the pose graph using g2o.
///
/// When g2o is unavailable (no SLAMFORGE_HAS_G2O), this is a no-op.
class PoseGraphOptimizer {
public:
    explicit PoseGraphOptimizer(const PoseGraphConfig& config = {});
    ~PoseGraphOptimizer();

    // Non-copyable
    PoseGraphOptimizer(const PoseGraphOptimizer&) = delete;
    PoseGraphOptimizer& operator=(const PoseGraphOptimizer&) = delete;

    /// @brief Run pose graph optimization on the map.
    ///
    /// @param map        The SLAM map containing all keyframes.
    /// @param loop_kf    The keyframe that triggered the loop closure.
    /// @param loop_kf_matched  The matched old keyframe.
    /// @param relative_pose  Relative SE(3) from matched to loop KF.
    /// @param information  6x6 information matrix for the loop edge.
    void Optimize(Map& map, std::shared_ptr<KeyFrame> loop_kf,
                  std::shared_ptr<KeyFrame> loop_kf_matched, const SE3& relative_pose,
                  const Mat6& information);

    /// @brief Optimizer settings in effect for this instance.
    const PoseGraphConfig& Config() const noexcept { return config_; }

private:
    PoseGraphConfig config_;
};

}  // namespace loop_closing
}  // namespace slamforge
