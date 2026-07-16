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

/// A verified correction mapping the drifted current map region into the
/// coordinate system of an older, matched region.
struct LoopCorrection {
    FrameId matched_frame{0};
    FrameId current_frame{0};
    geometry::Sim3 transform;
    int num_inliers = 0;
};

/// Smoothly interpolate a loop correction according to temporal frame ID.
geometry::Sim3 InterpolateLoopCorrection(const LoopCorrection& correction, FrameId frame);

/// Apply a world-coordinate Sim(3) correction to a world-to-camera pose.
SE3 ApplyLoopCorrectionToPose(const SE3& Tcw, const geometry::Sim3& correction);

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
    void FuseMatchedMapPoints(const std::vector<MapPointId>& matched_mps_cur,
                              const std::vector<MapPointId>& matched_mps_cand, Map& map);
};

}  // namespace loop_closing
}  // namespace slamforge
