// =============================================================================
// SLAMForge configuration loader — YAML parsing implementation
// =============================================================================

#include "slamforge/core/config.h"

#include <fstream>
#include <sstream>

#ifdef SLAMFORGE_HAS_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif

namespace slamforge {

SystemConfig SystemConfig::Default() {
    SystemConfig cfg;

    // KITTI defaults
    cfg.camera.width = 1241;
    cfg.camera.height = 376;
    cfg.camera.fx = 718.856;
    cfg.camera.fy = 718.856;
    cfg.camera.cx = 607.193;
    cfg.camera.cy = 185.216;

    cfg.orb.num_features = 1200;
    cfg.orb.scale_factor = 1.2;
    cfg.orb.num_levels = 8;
    cfg.orb.ini_threshold = 20;
    cfg.orb.min_threshold = 7;
    cfg.orb.patch_size = 31;

    cfg.tracking.min_features_for_tracking = 50;
    cfg.tracking.max_frames_between_kf = 40;
    cfg.tracking.min_frames_between_kf = 5;
    cfg.tracking.max_reprojection_error = 4.0;
    cfg.tracking.min_parallax_deg = 1.0;

    cfg.mapping.sliding_window_size = 20;
    cfg.mapping.min_observations = 3;
    cfg.mapping.max_reprojection_error = 4.0;

    cfg.loop_closing.enabled = false;
    cfg.loop_closing.min_similarity_score = 0.3;
    cfg.loop_closing.fallback_min_match_ratio = 0.03;
    cfg.loop_closing.min_consecutive_loops = 3;
    cfg.loop_closing.min_frame_separation = 500;
    cfg.loop_closing.pose_graph_iterations = 20;
    cfg.loop_closing.global_ba_iterations = 20;
    cfg.loop_closing.enable_global_ba = true;

    cfg.enable_viewer = true;
    cfg.enable_logging = true;
    cfg.log_level = 2;
    cfg.output_format = "tum";

    return cfg;
}

std::optional<SystemConfig> SystemConfig::LoadFromYAML(const std::string& path) {
#ifdef SLAMFORGE_HAS_YAML_CPP
    try {
        YAML::Node root = YAML::LoadFile(path);
        SystemConfig cfg = Default();

        // ── Camera ────────────────────────────────────────────────────────
        if (root["camera"]) {
            auto cam = root["camera"];
            cfg.camera.width = cam["width"].as<int>(cfg.camera.width);
            cfg.camera.height = cam["height"].as<int>(cfg.camera.height);
            cfg.camera.fx = cam["fx"].as<double>(cfg.camera.fx);
            cfg.camera.fy = cam["fy"].as<double>(cfg.camera.fy);
            cfg.camera.cx = cam["cx"].as<double>(cfg.camera.cx);
            cfg.camera.cy = cam["cy"].as<double>(cfg.camera.cy);
            cfg.camera.k1 = cam["k1"].as<double>(cfg.camera.k1);
            cfg.camera.k2 = cam["k2"].as<double>(cfg.camera.k2);
            cfg.camera.p1 = cam["p1"].as<double>(cfg.camera.p1);
            cfg.camera.p2 = cam["p2"].as<double>(cfg.camera.p2);
            cfg.camera.k3 = cam["k3"].as<double>(cfg.camera.k3);
        }

        // ── ORB ───────────────────────────────────────────────────────────
        if (root["orb"]) {
            auto orb = root["orb"];
            cfg.orb.num_features = orb["num_features"].as<int>(cfg.orb.num_features);
            cfg.orb.scale_factor = orb["scale_factor"].as<double>(cfg.orb.scale_factor);
            cfg.orb.num_levels = orb["num_levels"].as<int>(cfg.orb.num_levels);
            cfg.orb.ini_threshold = orb["ini_threshold"].as<int>(cfg.orb.ini_threshold);
            cfg.orb.min_threshold = orb["min_threshold"].as<int>(cfg.orb.min_threshold);
            cfg.orb.patch_size = orb["patch_size"].as<int>(cfg.orb.patch_size);
        }

        // ── Tracking ──────────────────────────────────────────────────────
        if (root["tracking"]) {
            auto trk = root["tracking"];
            cfg.tracking.min_features_for_tracking =
                trk["min_features_for_tracking"].as<int>(cfg.tracking.min_features_for_tracking);
            cfg.tracking.max_frames_between_kf =
                trk["max_frames_between_kf"].as<int>(cfg.tracking.max_frames_between_kf);
            cfg.tracking.min_frames_between_kf =
                trk["min_frames_between_kf"].as<int>(cfg.tracking.min_frames_between_kf);
            cfg.tracking.max_reprojection_error =
                trk["max_reprojection_error"].as<double>(cfg.tracking.max_reprojection_error);
            cfg.tracking.min_parallax_deg =
                trk["min_parallax_deg"].as<double>(cfg.tracking.min_parallax_deg);
        }

        // ── Mapping ───────────────────────────────────────────────────────
        if (root["mapping"]) {
            auto map = root["mapping"];
            cfg.mapping.sliding_window_size =
                map["sliding_window_size"].as<int>(cfg.mapping.sliding_window_size);
            cfg.mapping.min_observations =
                map["min_observations"].as<int>(cfg.mapping.min_observations);
            cfg.mapping.max_reprojection_error =
                map["max_reprojection_error"].as<double>(cfg.mapping.max_reprojection_error);
        }

        // ── Loop Closing ──────────────────────────────────────────────────
        if (root["loop_closing"]) {
            auto lc = root["loop_closing"];
            cfg.loop_closing.enabled = lc["enabled"].as<bool>(cfg.loop_closing.enabled);
            cfg.loop_closing.min_similarity_score =
                lc["min_similarity_score"].as<double>(cfg.loop_closing.min_similarity_score);
            cfg.loop_closing.fallback_min_match_ratio = lc["fallback_min_match_ratio"].as<double>(
                cfg.loop_closing.fallback_min_match_ratio);
            cfg.loop_closing.min_consecutive_loops =
                lc["min_consecutive_loops"].as<int>(cfg.loop_closing.min_consecutive_loops);
            cfg.loop_closing.min_frame_separation =
                lc["min_frame_separation"].as<int>(cfg.loop_closing.min_frame_separation);
            if (lc["vocab_path"]) {
                cfg.loop_closing.vocab_path = lc["vocab_path"].as<std::string>();
            }
            cfg.loop_closing.pose_graph_iterations =
                lc["pose_graph_iterations"].as<int>(cfg.loop_closing.pose_graph_iterations);
            cfg.loop_closing.global_ba_iterations =
                lc["global_ba_iterations"].as<int>(cfg.loop_closing.global_ba_iterations);
            cfg.loop_closing.enable_global_ba =
                lc["enable_global_ba"].as<bool>(cfg.loop_closing.enable_global_ba);
        }

        // ── General ───────────────────────────────────────────────────────
        if (root["output_trajectory"]) {
            cfg.output_trajectory = root["output_trajectory"].as<std::string>();
        }
        if (root["output_format"]) {
            cfg.output_format = root["output_format"].as<std::string>();
        }
        if (root["input_video"]) {
            cfg.input_video = root["input_video"].as<std::string>();
        }
        if (root["enable_viewer"]) {
            cfg.enable_viewer = root["enable_viewer"].as<bool>(cfg.enable_viewer);
        }
        if (root["enable_logging"]) {
            cfg.enable_logging = root["enable_logging"].as<bool>(cfg.enable_logging);
        }
        if (root["log_level"]) {
            cfg.log_level = root["log_level"].as<int>(cfg.log_level);
        }

        return cfg;
    } catch (const YAML::Exception& e) {
        // Config load failed — caller should handle
        return std::nullopt;
    }
#else
    (void)path;
    return Default();
#endif
}

}  // namespace slamforge
