// =============================================================================
// SLAMForge offline dense surface reconstruction
// =============================================================================

#pragma once

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "slamforge/core/config.h"
#include "slamforge/core/types.h"

namespace slamforge {

class Camera;
class Map;

namespace mapping {

/// A sparse landmark depth paired with the learned model value at the same
/// pixel. Public so calibration can be regression-tested without a model.
struct DepthAnchor {
    double prediction = 0.0;
    double camera_depth = 0.0;
};

/// Robust mapping from a relative learned prediction to SLAM camera depth.
struct DepthCalibration {
    enum class Mode { kInvalid, kInverseDepth, kDirectDepth };

    Mode mode = Mode::kInvalid;
    double scale = 0.0;
    double offset = 0.0;
    double median_relative_error = 1.0;
    double minimum_depth = 0.0;
    double maximum_depth = 0.0;
    int inlier_count = 0;

    [[nodiscard]] bool valid() const noexcept { return mode != Mode::kInvalid; }
    [[nodiscard]] double Evaluate(double prediction) const noexcept;
};

/// Fit both direct-depth and inverse-depth hypotheses and retain the one that
/// best agrees with the sparse SLAM landmarks.
[[nodiscard]] DepthCalibration CalibrateRelativeDepth(const std::vector<DepthAnchor>& anchors,
                                                      int minimum_samples,
                                                      double maximum_median_error);

struct DenseReconstructionResult {
    bool success = false;
    std::size_t point_count = 0;
    int selected_keyframes = 0;
    int calibrated_keyframes = 0;
    double voxel_size = 0.0;
    std::string error;

    explicit operator bool() const noexcept { return success; }
};

/// Offline dense mapper combining learned monocular depth with the final SLAM
/// poses and sparse landmarks. Learned predictions are scale/shift calibrated
/// independently at each selected keyframe, checked in adjacent views, and
/// fused into a deterministic colored voxel cloud.
class DenseMapper {
public:
    using ProgressCallback = std::function<void(int current, int total)>;

    DenseMapper(const Camera& camera, DenseMappingConfig config = {});

    [[nodiscard]] static bool RuntimeAvailable() noexcept;

    [[nodiscard]] DenseReconstructionResult Reconstruct(
        Map& map, const std::filesystem::path& model_path,
        const std::filesystem::path& output_path,
        const ProgressCallback& progress = ProgressCallback{}) const;

private:
    const Camera& camera_;
    DenseMappingConfig config_;
};

}  // namespace mapping
}  // namespace slamforge
