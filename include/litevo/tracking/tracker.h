// =============================================================================
// Tracker — monocular visual SLAM tracking frontend
// =============================================================================

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "litevo/core/config.h"
#include "litevo/core/map.h"
#include "litevo/core/types.h"
#include "litevo/tracking/initializer.h"

namespace litevo {

class Camera;
class Frame;
class KeyFrame;

namespace features {
class OrbExtractor;
}

namespace mapping {
class LocalMapper;
}

namespace tracking {

class FeatureMatcher;

/// Tracking state machine.
enum class TrackingState : std::uint8_t {
    NOT_INITIALIZED,  ///< Waiting for successful initialization
    INITIALIZING,     ///< Attempting two-view initialization
    OK,               ///< Tracking normally
    LOST              ///< Tracking lost, attempting relocalization
};

/// @brief Monocular visual SLAM tracking frontend.
///
/// Processes frames sequentially, estimating camera pose through:
///   1. Two-view initialization (H/F model selection)
///   2. Constant-velocity motion model tracking
///   3. Reference keyframe tracking (fallback)
///   4. Local map refinement
///   5. Relocalization on tracking loss
///
/// Thread-safe for single-threaded use. Thread safety for multi-threaded
/// operation (with Local Mapping) will be added in Phase 4.
class Tracker {
public:
    /// @param camera      Camera model (shared, read-only).
    /// @param config      Tracking configuration.
    /// @param orb_config  ORB extractor configuration.
    Tracker(const Camera& camera, const TrackingConfig& config, const OrbConfig& orb_config);

    ~Tracker();

    /// @brief Process a new frame. Main entry point.
    /// @param image      Grayscale image (CV_8UC1).
    /// @param timestamp  Frame timestamp in seconds.
    /// @return Estimated camera pose Tcw, or std::nullopt if lost.
    std::optional<SE3> Track(const cv::Mat& image, double timestamp);

    // ── State ──────────────────────────────────────────────────────────────

    TrackingState State() const { return state_; }
    bool IsInitialized() const { return state_ == TrackingState::OK; }
    bool IsLost() const { return state_ == TrackingState::LOST; }

    // ── Access ─────────────────────────────────────────────────────────────

    const Map& GetMap() const { return map_; }
    Map& GetMap() { return map_; }
    const Frame* CurrentFrame() const { return current_frame_.get(); }
    int NumTrackedPoints() const { return num_tracked_points_; }
    int NumKeyFrames() const { return static_cast<int>(map_.KeyFrameCount()); }

    /// @brief Reset tracker to initial state.
    void Reset();

    /// @brief Set the local mapper for asynchronous keyframe processing.
    /// Set to nullptr to disable (single-threaded mode).
    void SetLocalMapper(mapping::LocalMapper* mapper) { local_mapper_ = mapper; }

    /// @brief Get the local mapper, or nullptr if none set.
    mapping::LocalMapper* GetLocalMapper() const { return local_mapper_; }

private:
    // ── Configuration ──────────────────────────────────────────────────────

    const Camera& camera_;
    TrackingConfig config_;
    OrbConfig orb_config_;

    // ── Components ─────────────────────────────────────────────────────────

    std::unique_ptr<features::OrbExtractor> orb_extractor_;
    std::unique_ptr<features::OrbExtractor> orb_extractor_ini_;
    std::unique_ptr<FeatureMatcher> matcher_;
    std::unique_ptr<MonocularInitializer> initializer_;
    Map map_;

    // ── State ──────────────────────────────────────────────────────────────

    TrackingState state_ = TrackingState::NOT_INITIALIZED;
    std::shared_ptr<Frame> current_frame_;
    std::shared_ptr<Frame> last_frame_;
    std::shared_ptr<KeyFrame> reference_kf_;

    SE3 velocity_{SE3::Identity()};
    bool has_velocity_ = false;

    // ── Bookkeeping ────────────────────────────────────────────────────────

    int num_tracked_points_ = 0;
    int frames_since_last_kf_ = 0;
    bool is_new_keyframe_ = false;

    mapping::LocalMapper* local_mapper_ = nullptr;

    // ── Tracking sub-methods ───────────────────────────────────────────────

    /// @brief Predict pose from constant-velocity model, project last frame's
    ///        map points into current frame, and optimize via PnP.
    bool TrackWithMotionModel();

    /// @brief Match descriptors against reference keyframe, then PnP.
    bool TrackReferenceKeyFrame();

    /// @brief Project local map points and refine pose.
    bool TrackLocalMap();

    /// @brief Decide whether the current frame should become a keyframe.
    bool NeedNewKeyFrame();

    /// @brief Brute-force descriptor matching against recent keyframes.
    bool Relocalization();

    /// @brief Create the initial map from a successful initialization.
    void CreateInitialMap(const InitializationResult& result);

    /// @brief Update the constant-velocity motion model.
    void UpdateMotionModel();

    /// @brief Clean map points that haven't been found recently.
    void CleanMapPoints();

    /// @brief Run PnP with RANSAC + refinement on 3D-2D correspondences.
    bool EstimatePose(const std::vector<cv::Point3f>& pts_3d,
                      const std::vector<cv::Point2f>& pts_2d, SE3& Tcw, int min_inliers = 10);

    /// @brief Build OpenCV intrinsic matrix.
    cv::Mat BuildK() const;
};

}  // namespace tracking
}  // namespace litevo
