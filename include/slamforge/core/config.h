// =============================================================================
// SLAMForge configuration system — YAML-based runtime parameter management
// =============================================================================

#pragma once

#include <optional>
#include <string>

namespace slamforge {

// ── Camera configuration ─────────────────────────────────────────────────────

/// @brief Pinhole camera intrinsic parameters.
struct CameraConfig {
    int width = 1241;     ///< Image width in pixels
    int height = 376;     ///< Image height in pixels
    double fx = 718.856;  ///< Focal length x
    double fy = 718.856;  ///< Focal length y
    double cx = 607.193;  ///< Principal point x
    double cy = 185.216;  ///< Principal point y
    double k1 = 0.0;      ///< Radial distortion coefficient 1
    double k2 = 0.0;      ///< Radial distortion coefficient 2
    double p1 = 0.0;      ///< Tangential distortion coefficient 1
    double p2 = 0.0;      ///< Tangential distortion coefficient 2
    double k3 = 0.0;      ///< Radial distortion coefficient 3
};

// ── ORB feature configuration ────────────────────────────────────────────────

/// @brief ORB feature extractor parameters.
struct OrbConfig {
    int num_features = 1200;    ///< Target number of features per frame
    double scale_factor = 1.2;  ///< Pyramid scale factor between levels
    int num_levels = 8;         ///< Number of pyramid levels
    int ini_threshold = 20;     ///< FAST initial threshold
    int min_threshold = 7;      ///< FAST minimum threshold
    int patch_size = 31;        ///< ORB descriptor patch size
};

// ── Tracking configuration ───────────────────────────────────────────────────

/// @brief Tracking thread parameters.
struct TrackingConfig {
    int min_features_for_tracking = 50;   ///< Min tracked features to stay alive
    int max_frames_between_kf = 40;       ///< Max frames before forcing keyframe
    int min_frames_between_kf = 5;        ///< Min frames between keyframes
    double max_reprojection_error = 4.0;  ///< Max reprojection error (pixels)
    double min_parallax_deg = 1.0;        ///< Min parallax for triangulation
};

// ── Mapping configuration ────────────────────────────────────────────────────

/// @brief Local mapping thread parameters.
struct MappingConfig {
    int sliding_window_size = 20;         ///< Keyframes in local BA window
    int min_observations = 3;             ///< Min observations to keep a map point
    double max_reprojection_error = 4.0;  ///< Max reprojection for map points
};

// ── Loop closing configuration ───────────────────────────────────────────────

/// @brief Loop closing thread parameters.
struct LoopClosingConfig {
    bool enabled = false;               ///< Enable loop closing (needs vocabulary)
    double min_similarity_score = 0.3;  ///< Minimum bag-of-words similarity
    int min_consecutive_loops = 3;      ///< Consecutive detections to accept
    std::string vocab_path;             ///< Path to ORB vocabulary file
    int pose_graph_iterations = 20;     ///< Max iterations for pose graph optimization
    int global_ba_iterations = 20;      ///< Max iterations for global BA
    bool enable_global_ba = true;       ///< Run global BA after loop closure
};

// ── System configuration ─────────────────────────────────────────────────────

/// @brief Top-level SLAMForge system configuration.
struct SystemConfig {
    CameraConfig camera;
    OrbConfig orb;
    TrackingConfig tracking;
    MappingConfig mapping;
    LoopClosingConfig loop_closing;

    /// Input / output.
    std::string input_video;
    std::string output_trajectory;
    std::string output_format = "tum";  ///< "tum", "kitti", "euroc"

    /// Performance.
    bool enable_viewer = true;   ///< Enable real-time visualization
    bool enable_logging = true;  ///< Enable structured logging
    int log_level = 2;           ///< 0=trace, 1=debug, 2=info, 3=warn, 4=error

    /// @brief Load configuration from a YAML file.
    /// @param path Path to the YAML configuration file.
    /// @return Config object, or std::nullopt on failure.
    static std::optional<SystemConfig> LoadFromYAML(const std::string& path);

    /// @brief Create default configuration (KITTI dataset).
    static SystemConfig Default();
};

}  // namespace slamforge
