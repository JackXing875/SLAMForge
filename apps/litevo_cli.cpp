// =============================================================================
// LiteVO CLI — command-line monocular visual SLAM
// =============================================================================

#include <CLI/CLI.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "litevo/litevo.h"

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    CLI::App app{"LiteVO — Industrial-Grade Monocular Visual SLAM"};

    std::string config_path = "config/default.yaml";
    std::string input_path;
    std::string output_path = "trajectory.txt";
    bool verbose = false;

    app.add_option("-c,--config", config_path, "YAML configuration file")->check(CLI::ExistingFile);
    app.add_option("-i,--input", input_path, "Image directory")->required();
    app.add_option("-o,--output", output_path, "Output trajectory (TUM format)");
    app.add_flag("-v,--verbose", verbose, "Verbose per-frame output");

    CLI11_PARSE(app, argc, argv);

    // Load configuration
    auto cfg_opt = litevo::SystemConfig::LoadFromYAML(config_path);
    if (!cfg_opt) {
        std::cerr << "Error: failed to load config from " << config_path << "\n";
        return EXIT_FAILURE;
    }
    auto cfg = *cfg_opt;

    // Build camera model
    litevo::Camera::CameraParams cam_params{cfg.camera.fx,    cfg.camera.fy,    cfg.camera.cx,
                                            cfg.camera.cy,    cfg.camera.k1,    cfg.camera.k2,
                                            cfg.camera.p1,    cfg.camera.p2,    cfg.camera.k3,
                                            cfg.camera.width, cfg.camera.height};
    litevo::Camera camera = litevo::Camera::FromParams(cam_params);

    // Create tracker
    litevo::tracking::Tracker tracker(camera, cfg.tracking, cfg.orb);

    // Create local mapper (background thread)
    litevo::features::OrbExtractor::Options mapper_orb_opts;
    mapper_orb_opts.num_features = cfg.orb.num_features;
    mapper_orb_opts.scale_factor = static_cast<double>(cfg.orb.scale_factor);
    mapper_orb_opts.num_levels = cfg.orb.num_levels;
    mapper_orb_opts.ini_threshold = cfg.orb.ini_threshold;
    mapper_orb_opts.min_threshold = cfg.orb.min_threshold;
    mapper_orb_opts.patch_size = cfg.orb.patch_size;
    litevo::features::OrbExtractor mapper_extractor(mapper_orb_opts);

    litevo::mapping::LocalMapper local_mapper(tracker.GetMap(), camera, cfg.mapping,
                                              mapper_extractor);
    tracker.SetLocalMapper(&local_mapper);

    // Collect images from directory
    fs::path input_fs(input_path);
    if (!fs::is_directory(input_fs)) {
        std::cerr << "Error: input must be a directory of images\n";
        return EXIT_FAILURE;
    }

    std::vector<std::string> image_paths;
    for (const auto& entry : fs::directory_iterator(input_fs)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".ppm") {
            image_paths.push_back(entry.path().string());
        }
    }
    std::sort(image_paths.begin(), image_paths.end());

    if (image_paths.empty()) {
        std::cerr << "Error: no images found in " << input_path << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "LiteVO v" << litevo::kVersionString << "\n";
    std::cout << "Input:  " << input_path << " (" << image_paths.size() << " images)\n";
    std::cout << "Output: " << output_path << "\n";
    std::cout << "Camera: " << cfg.camera.width << "x" << cfg.camera.height
              << "  fx=" << cfg.camera.fx << "\n\n";

    // Open trajectory file
    std::ofstream traj_file(output_path);
    if (!traj_file.is_open()) {
        std::cerr << "Error: cannot open " << output_path << "\n";
        return EXIT_FAILURE;
    }

    // Start local mapper
    local_mapper.Start();
    std::cout << "Local mapping thread started\n\n";

    // ── Main tracking loop ────────────────────────────────────────────────

    int frame_count = 0;
    int lost_count = 0;
    double timestamp = 0.0;
    const double dt = 1.0 / 30.0;  // Assume 30 Hz

    for (const auto& path : image_paths) {
        cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
        if (image.empty()) {
            if (verbose) {
                std::cerr << "Warning: cannot read " << path << "\n";
            }
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        auto pose_opt = tracker.Track(gray, timestamp);

        if (pose_opt) {
            // TUM format: timestamp tx ty tz qx qy qz qw (Twc = world from camera)
            litevo::SE3 Twc = pose_opt->inverse();
            litevo::Pose p = litevo::Pose::FromSE3(Twc);

            traj_file << timestamp << " " << p.position.x() << " " << p.position.y() << " "
                      << p.position.z() << " " << p.orientation.x() << " " << p.orientation.y()
                      << " " << p.orientation.z() << " " << p.orientation.w() << "\n";
        } else {
            lost_count++;
        }

        if (verbose && (frame_count % 10 == 0 || pose_opt)) {
            auto state = tracker.State();
            std::cout << "Frame " << frame_count << " | state=";
            switch (state) {
                case litevo::tracking::TrackingState::NOT_INITIALIZED:
                case litevo::tracking::TrackingState::INITIALIZING:
                    std::cout << "INIT";
                    break;
                case litevo::tracking::TrackingState::OK:
                    std::cout << "OK";
                    break;
                case litevo::tracking::TrackingState::LOST:
                    std::cout << "LOST";
                    break;
            }
            std::cout << " | tracked=" << tracker.NumTrackedPoints()
                      << " | kfs=" << tracker.NumKeyFrames()
                      << " | mps=" << tracker.GetMap().MapPointCount() << "\n";
        }

        timestamp += dt;
        frame_count++;
    }

    traj_file.close();

    // Stop local mapper
    local_mapper.Stop();
    std::cout << "Local mapping thread stopped\n";

    // ── Summary ───────────────────────────────────────────────────────────
    std::cout << "\n══════════════════════════════════════════════\n";
    std::cout << "  Tracking complete\n";
    std::cout << "  Frames:     " << frame_count << "\n";
    std::cout << "  Lost:       " << lost_count << "\n";
    std::cout << "  Keyframes:  " << tracker.NumKeyFrames() << "\n";
    std::cout << "  Map points: " << tracker.GetMap().MapPointCount() << "\n";
    std::cout << "  Output:     " << output_path << "\n";
    std::cout << "══════════════════════════════════════════════\n";

    return EXIT_SUCCESS;
}
