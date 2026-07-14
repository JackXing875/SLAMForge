// =============================================================================
// Bundle Adjustment — Ceres-based local window optimization
// =============================================================================

#pragma once

#include <vector>

#include "litevo/core/types.h"

namespace litevo {

class Camera;
class KeyFrame;
class MapPoint;

namespace optimization {

/// @brief Configuration for LocalBundleAdjuster.
struct BAOptions {
    int max_iterations = 10;
    double huber_delta = 5.0;  ///< Huber loss threshold in pixels
    bool use_huber = true;
};

/// @brief Local bundle adjustment over a sliding window of keyframes.
///
/// Optimizes both camera poses and 3D map point positions to minimize
/// reprojection error. Fixed keyframes (outside the local window) are
/// held constant.
class LocalBundleAdjuster {
public:
    /// @param camera  Camera model (intrinsics read-only).
    explicit LocalBundleAdjuster(const Camera& camera);

    /// @param camera  Camera model (intrinsics read-only).
    /// @param opts    Solver options.
    LocalBundleAdjuster(const Camera& camera, const BAOptions& opts);

    /// @brief Run local bundle adjustment.
    int Optimize(std::vector<KeyFrame*>& local_kfs, std::vector<KeyFrame*>& fixed_kfs,
                 std::vector<MapPoint*>& map_points);

private:
    const Camera& camera_;
    BAOptions opts_;
};

}  // namespace optimization
}  // namespace litevo
