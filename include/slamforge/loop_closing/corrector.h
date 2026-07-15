// =============================================================================
// LoopCorrector — Sim(3) propagation and map point fusion after loop closure
// =============================================================================

#pragma once

#include <memory>
#include <set>
#include <vector>

#include "slamforge/core/types.h"
#include "slamforge/geometry/sim3.h"

namespace slamforge {

class KeyFrame;
class Map;

namespace loop_closing {

/// @brief Corrects the map after a loop closure is verified.
///
/// Operations:
///   1. Propagate Sim(3) correction from the loop keyframe through the
///      covisibility graph
///   2. Fuse duplicated map points on both sides of the loop
///   3. Update covisibility graph connections
class LoopCorrector {
public:
    LoopCorrector() = default;

    /// @brief Apply loop correction to the entire map.
    ///
    /// @param current_kf      Current keyframe that closes the loop.
    /// @param matched_kf      Matched (old) keyframe from loop detection.
    /// @param S_cw_correct    Sim(3) correction for current_kf.
    /// @param matches         2D-2D matches between current and matched KFs.
    /// @param matched_mps_cur  MapPoint IDs matched in current KF.
    /// @param matched_mps_cand MapPoint IDs matched in candidate KF.
    /// @param map              The global map to update.
    void CorrectLoop(std::shared_ptr<KeyFrame> current_kf, std::shared_ptr<KeyFrame> matched_kf,
                     const geometry::Sim3& S_cw_correct,
                     const std::vector<std::pair<int, int>>& matches,
                     const std::vector<MapPointId>& matched_mps_cur,
                     const std::vector<MapPointId>& matched_mps_cand, Map& map);

private:
    /// @brief Propagate Sim(3) correction along the covisibility graph.
    /// Starting from the current KF, recursively apply to neighbors.
    void PropagateCorrection(KeyFrame* kf, const geometry::Sim3& S_correction,
                             std::set<KeyFrame*>& corrected);

    /// @brief Fuse duplicated map points between loop-connected keyframes.
    /// Searches for map points that represent the same 3D point and merges them.
    void SearchAndFuse(const std::vector<std::shared_ptr<KeyFrame>>& loop_kfs,
                       const std::vector<MapPointId>& matched_mps, double max_reproj_error,
                       Map& map);

    /// @brief Update covisibility connections after loop correction.
    void UpdateConnections(Map& map);
};

}  // namespace loop_closing
}  // namespace slamforge
