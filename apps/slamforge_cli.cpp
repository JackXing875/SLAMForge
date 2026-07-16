// =============================================================================
// SLAMForge CLI — command-line monocular visual SLAM
// =============================================================================
// Subcommand-based CLI using CLI11:
//   slamforge run     — Run SLAM on images or video
//   slamforge eval    — Evaluate trajectory against ground truth
//   slamforge bench   — Batch-run SLAM on dataset sequences
// =============================================================================

#include <Eigen/SVD>

#include <CLI/CLI.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#ifndef _WIN32
#include <sys/wait.h>
#endif

#include "slamforge/slamforge.h"

namespace fs = std::filesystem;

// =============================================================================
// Image / video input helpers
// =============================================================================

namespace {

fs::path executable_directory;

/// Resolve the binary directory even when the command was found through PATH.
/// The installed Python evaluation tools are located relative to this path.
fs::path ResolveExecutableDirectory(const char* argv0) {
    std::error_code error;
    const fs::path invocation = argv0 ? fs::path(argv0) : fs::path{};
    if (!invocation.empty() && invocation.has_parent_path()) {
        const fs::path absolute = fs::absolute(invocation, error);
        if (!error) {
            return absolute.parent_path();
        }
    }

    const char* path_value = std::getenv("PATH");
    if (path_value != nullptr && !invocation.empty()) {
#ifdef _WIN32
        constexpr char path_separator = ';';
#else
        constexpr char path_separator = ':';
#endif
        std::istringstream path_entries(path_value);
        std::string directory;
        while (std::getline(path_entries, directory, path_separator)) {
            const fs::path candidate = fs::path(directory.empty() ? "." : directory) / invocation;
            if (fs::is_regular_file(candidate, error) && !error) {
                const fs::path absolute = fs::absolute(candidate, error);
                if (!error) {
                    return absolute.parent_path();
                }
            }
            error.clear();
        }
    }

    const fs::path working_directory = fs::current_path(error);
    return error ? fs::path{} : working_directory;
}

/// Check whether a file path has a recognized video extension.
bool IsVideoFile(const std::string& path) {
    std::string ext = fs::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext == ".mp4" || ext == ".avi" || ext == ".mov" || ext == ".mkv" || ext == ".m4v";
}

/// Collect and sort image paths from a directory without allowing filesystem
/// exceptions to terminate the CLI on an inaccessible input directory.
bool GetImagePaths(const std::string& dir_path, std::vector<std::string>& paths,
                   std::string& error) {
    paths.clear();
    std::error_code filesystem_error;
    if (!fs::is_directory(dir_path, filesystem_error)) {
        error = filesystem_error ? "cannot access input directory '" + dir_path +
                                       "': " + filesystem_error.message()
                                 : "input must be an image directory or video file";
        return false;
    }

    fs::directory_iterator iterator(dir_path, filesystem_error);
    const fs::directory_iterator end;
    if (filesystem_error) {
        error =
            "cannot enumerate input directory '" + dir_path + "': " + filesystem_error.message();
        return false;
    }

    while (iterator != end) {
        const fs::directory_entry entry = *iterator;
        const bool is_regular = entry.is_regular_file(filesystem_error);
        if (filesystem_error) {
            error = "cannot inspect input entry '" + entry.path().string() +
                    "': " + filesystem_error.message();
            return false;
        }
        if (!is_regular) {
            iterator.increment(filesystem_error);
            if (filesystem_error) {
                error = "cannot enumerate input directory '" + dir_path +
                        "': " + filesystem_error.message();
                return false;
            }
            continue;
        }
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".ppm") {
            paths.push_back(entry.path().string());
        }
        iterator.increment(filesystem_error);
        if (filesystem_error) {
            error = "cannot enumerate input directory '" + dir_path +
                    "': " + filesystem_error.message();
            return false;
        }
    }
    std::sort(paths.begin(), paths.end());
    return true;
}

/// Normalize timestamp units at the input boundary.  EuRoC CSV files use
/// nanoseconds, while Frame and the CLI use seconds.
double TimestampSeconds(double timestamp) {
    return std::abs(timestamp) > 1e12 ? timestamp * 1e-9 : timestamp;
}

/// Load timestamps for an image sequence.
///
/// Accepted records are either ``timestamp`` (one record per sorted image) or
/// ``timestamp filename`` / ``timestamp,filename``.  The latter supports the
/// standard TUM rgb.txt and EuRoC data.csv layouts, including relative paths.
bool LoadImageTimestamps(const std::string& path, const std::vector<std::string>& image_paths,
                         std::vector<double>& timestamps, std::string& error) {
    std::ifstream file(path);
    if (!file.is_open()) {
        error = "cannot open timestamp file '" + path + "'";
        return false;
    }

    std::vector<double> ordered_timestamps;
    std::unordered_map<std::string, double> timestamps_by_name;
    std::string line;
    int line_number = 0;

    while (std::getline(file, line)) {
        ++line_number;
        const auto first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos || line[first] == '#') {
            continue;
        }

        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream record(line);
        std::string timestamp_token;
        std::string filename;
        if (!(record >> timestamp_token)) {
            continue;
        }

        double timestamp = 0.0;
        try {
            size_t parsed = 0;
            timestamp = std::stod(timestamp_token, &parsed);
            if (parsed != timestamp_token.size()) {
                throw std::invalid_argument("trailing characters");
            }
        } catch (const std::exception&) {
            error = "invalid timestamp at " + path + ":" + std::to_string(line_number);
            return false;
        }
        if (!std::isfinite(timestamp)) {
            error = "non-finite timestamp at " + path + ":" + std::to_string(line_number);
            return false;
        }

        timestamp = TimestampSeconds(timestamp);
        if (record >> filename) {
            timestamps_by_name[fs::path(filename).filename().string()] = timestamp;
        } else {
            ordered_timestamps.push_back(timestamp);
        }
    }

    if (!timestamps_by_name.empty()) {
        if (!ordered_timestamps.empty()) {
            error = "timestamp file mixes named and positional records: '" + path + "'";
            return false;
        }
        timestamps.reserve(image_paths.size());
        for (const auto& image_path : image_paths) {
            const auto it = timestamps_by_name.find(fs::path(image_path).filename().string());
            if (it == timestamps_by_name.end()) {
                error = "no timestamp for image '";
                error += image_path;
                error += "' in '";
                error += path;
                error += "'";
                return false;
            }
            timestamps.push_back(it->second);
        }
        return true;
    }

    if (ordered_timestamps.size() != image_paths.size()) {
        error = "timestamp count (" + std::to_string(ordered_timestamps.size()) +
                ") does not match image count (" + std::to_string(image_paths.size()) + ")";
        return false;
    }
    timestamps = std::move(ordered_timestamps);
    return true;
}

}  // anonymous namespace

// =============================================================================
// Python tool runner (for eval subcommand)
// =============================================================================

/// Normalize std::system's platform-specific status into a process exit code.
static int NormalizeSystemStatus(int status) {
    if (status == -1) {
        return EXIT_FAILURE;
    }
#ifdef _WIN32
    return status;
#else
    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }
    return EXIT_FAILURE;
#endif
}

/// Shell out to a Python evaluation script in tools.
/// @returns the child process's normalized exit code (0 on success).
static std::string ShellQuote(const std::string& value) {
    std::string quoted = "'";
    for (const char ch : value) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted += ch;
        }
    }
    quoted += "'";
    return quoted;
}

static int RunPythonTool(const std::string& tool_name, const std::vector<std::string>& args) {
    std::vector<fs::path> tool_candidates = {fs::path("tools") / tool_name};
#ifdef SLAMFORGE_TOOLS_RELATIVE_TO_BINDIR
    if (!executable_directory.empty()) {
        tool_candidates.push_back(executable_directory /
                                  fs::path(SLAMFORGE_TOOLS_RELATIVE_TO_BINDIR) / tool_name);
    }
#endif

    fs::path tool_path;
    for (const auto& candidate : tool_candidates) {
        std::error_code error;
        if (fs::is_regular_file(candidate, error) && !error) {
            tool_path = candidate;
            break;
        }
    }
    if (tool_path.empty()) {
        std::cerr << "Error: cannot locate Python evaluation tool '" << tool_name << "'\n";
        return EXIT_FAILURE;
    }

    std::string cmd = "python3 " + ShellQuote(tool_path.string());
    for (const auto& arg : args) {
        cmd += " " + ShellQuote(arg);
    }

    std::cout << "  Running: " << cmd << "\n";
    return NormalizeSystemStatus(std::system(cmd.c_str()));
}

// =============================================================================
// Trajectory I/O and inline ATE / RPE computation
// =============================================================================

/// Lightweight trajectory representation (translations only).
struct TrajectoryData {
    std::vector<slamforge::Vec3> positions;
    std::vector<double> timestamps;
    bool has_timestamps = false;

    bool empty() const { return positions.empty(); }
    size_t size() const { return positions.size(); }
};

/// Load trajectory translations.  Supported formats: "tum", "kitti", "euroc".
static TrajectoryData LoadTrajectory(const std::string& path, const std::string& format) {
    TrajectoryData data;
    data.has_timestamps = (format != "kitti");
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open trajectory file: " << path << "\n";
        return data;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#')
            continue;

        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        double tx = 0.0, ty = 0.0, tz = 0.0;
        double timestamp = 0.0;
        bool valid = false;

        if (format == "kitti") {
            // KITTI pose: 3x4 matrix flattened row-major
            //   r11 r12 r13 tx  r21 r22 r23 ty  r31 r32 r33 tz
            double val = 0.0;
            for (int i = 0; i < 12; ++i) {
                if (!(iss >> val)) {
                    valid = false;
                    break;
                }
                if (i == 3)
                    tx = val;
                else if (i == 7)
                    ty = val;
                else if (i == 11)
                    tz = val;
                valid = true;
            }
        } else {
            // TUM: timestamp tx ty tz qx qy qz qw
            // EuRoC: timestamp,tx,ty,tz,qw,qx,qy,qz (commas normalized above)
            valid = static_cast<bool>(iss >> timestamp >> tx >> ty >> tz);
        }

        if (valid) {
            data.positions.push_back(slamforge::Vec3(tx, ty, tz));
            if (data.has_timestamps) {
                data.timestamps.push_back(TimestampSeconds(timestamp));
            }
        }
    }

    return data;
}

/// Associate timestamped trajectories one-to-one; timestamp-free formats use
/// positional correspondence (the only information available in KITTI files).
static void AssociateTrajectories(const TrajectoryData& estimate, const TrajectoryData& groundtruth,
                                  TrajectoryData& associated_estimate,
                                  TrajectoryData& associated_groundtruth,
                                  double max_time_difference = 0.02) {
    associated_estimate = {};
    associated_groundtruth = {};

    const auto append_pair = [&](size_t estimate_index, size_t groundtruth_index) {
        associated_estimate.positions.push_back(estimate.positions[estimate_index]);
        associated_groundtruth.positions.push_back(groundtruth.positions[groundtruth_index]);
    };

    if (!estimate.has_timestamps || !groundtruth.has_timestamps ||
        estimate.timestamps.size() != estimate.positions.size() ||
        groundtruth.timestamps.size() != groundtruth.positions.size()) {
        const size_t count = std::min(estimate.size(), groundtruth.size());
        for (size_t index = 0; index < count; ++index) {
            append_pair(index, index);
        }
        return;
    }

    std::vector<size_t> estimate_order(estimate.size());
    std::vector<size_t> groundtruth_order(groundtruth.size());
    std::iota(estimate_order.begin(), estimate_order.end(), 0);
    std::iota(groundtruth_order.begin(), groundtruth_order.end(), 0);
    std::sort(estimate_order.begin(), estimate_order.end(), [&](size_t lhs, size_t rhs) {
        return estimate.timestamps[lhs] < estimate.timestamps[rhs];
    });
    std::sort(groundtruth_order.begin(), groundtruth_order.end(), [&](size_t lhs, size_t rhs) {
        return groundtruth.timestamps[lhs] < groundtruth.timestamps[rhs];
    });

    size_t estimate_index = 0;
    size_t groundtruth_index = 0;
    while (estimate_index < estimate_order.size() && groundtruth_index < groundtruth_order.size()) {
        const size_t est = estimate_order[estimate_index];
        const size_t gt = groundtruth_order[groundtruth_index];
        const double delta = estimate.timestamps[est] - groundtruth.timestamps[gt];
        if (std::abs(delta) <= max_time_difference) {
            append_pair(est, gt);
            ++estimate_index;
            ++groundtruth_index;
        } else if (delta < 0.0) {
            ++estimate_index;
        } else {
            ++groundtruth_index;
        }
    }
}

/// Umeyama alignment: find scale, rotation R, and translation t that minimise
///   sum_i || scale * R * src_i + t - dst_i ||^2.
/// @param[out] aligned  src points transformed by the optimal (R, t).
/// @returns estimated scale.
static double AlignUmeyama(const std::vector<slamforge::Vec3>& src,
                           const std::vector<slamforge::Vec3>& dst,
                           std::vector<slamforge::Vec3>& aligned) {
    const size_t N = std::min(src.size(), dst.size());
    aligned.resize(N);
    if (N < 3) {
        // Too few points — just copy
        for (size_t i = 0; i < N; ++i)
            aligned[i] = src[i];
        return 1.0;
    }

    // Build Nx3 matrices
    Eigen::MatrixXd P(N, 3), Q(N, 3);
    for (size_t i = 0; i < N; ++i) {
        const auto row = static_cast<Eigen::Index>(i);
        P.row(row) = src[i].transpose();
        Q.row(row) = dst[i].transpose();
    }

    // Centroids
    Eigen::RowVector3d mean_p = P.colwise().mean();
    Eigen::RowVector3d mean_q = Q.colwise().mean();

    // Center the point clouds
    Eigen::MatrixXd Pc = P.rowwise() - mean_p;
    Eigen::MatrixXd Qc = Q.rowwise() - mean_q;

    // Cross-covariance
    Eigen::Matrix3d H = Pc.transpose() * Qc;

    // SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

    // Reflection correction
    if (R.determinant() < 0.0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1.0;
        R = V * svd.matrixU().transpose();
    }

    // Isotropic scale.  For row-vector point clouds the numerator is
    // tr(R * (Pc^T * Qc)); writing Qc^T * Pc * R is not equivalent in general.
    const double variance = Pc.squaredNorm();
    const double scale = variance > 1e-12 ? (R * H).trace() / variance : 1.0;

    // Translation
    Eigen::Vector3d t = mean_q.transpose() - scale * R * mean_p.transpose();

    // Apply
    for (size_t i = 0; i < N; ++i) {
        aligned[i] = scale * R * src[i] + t;
    }

    return scale;
}

/// Compute Absolute Trajectory Error RMSE (after Umeyama alignment).
static double ComputeATE(const TrajectoryData& est, const TrajectoryData& gt) {
    TrajectoryData associated_est;
    TrajectoryData associated_gt;
    AssociateTrajectories(est, gt, associated_est, associated_gt);
    const size_t N = associated_est.size();
    if (N == 0)
        return -1.0;

    std::vector<slamforge::Vec3> aligned;
    AlignUmeyama(associated_est.positions, associated_gt.positions, aligned);

    double sum_sq = 0.0;
    for (size_t i = 0; i < N; ++i) {
        sum_sq += (aligned[i] - associated_gt.positions[i]).squaredNorm();
    }
    return std::sqrt(sum_sq / static_cast<double>(N));
}

/// Compute translational Relative Pose Error RMSE (after Umeyama alignment).
/// Uses frame delta = 1.
static double ComputeRPE(const TrajectoryData& est, const TrajectoryData& gt, int delta = 1) {
    TrajectoryData associated_est;
    TrajectoryData associated_gt;
    AssociateTrajectories(est, gt, associated_est, associated_gt);
    const size_t N = associated_est.size();
    if (delta <= 0)
        return -1.0;
    const size_t step = static_cast<size_t>(delta);
    if (N <= step || N == 0)
        return -1.0;

    // Align the full trajectory first
    std::vector<slamforge::Vec3> est_aligned;
    AlignUmeyama(associated_est.positions, associated_gt.positions, est_aligned);

    double sum_sq = 0.0;
    size_t count = 0;
    for (size_t i = 0; i + step < N; ++i) {
        slamforge::Vec3 est_delta = est_aligned[i + step] - est_aligned[i];
        slamforge::Vec3 gt_delta = associated_gt.positions[i + step] - associated_gt.positions[i];
        sum_sq += (est_delta - gt_delta).squaredNorm();
        ++count;
    }
    if (count == 0)
        return -1.0;
    return std::sqrt(sum_sq / static_cast<double>(count));
}

/// Validate the trajectory output encoding configured by the user.
static bool IsTrajectoryFormat(const std::string& format) {
    return format == "tum" || format == "kitti" || format == "euroc";
}

/// Write a world-from-camera pose in a documented dataset trajectory format.
static bool WriteTrajectoryPose(std::ostream& stream, const slamforge::SE3& Tcw, double timestamp,
                                const std::string& format) {
    const slamforge::SE3 Twc = Tcw.inverse();
    stream << std::fixed << std::setprecision(9);

    if (format == "tum") {
        const slamforge::Pose pose = slamforge::Pose::FromSE3(Twc);
        stream << timestamp << " " << pose.position.x() << " " << pose.position.y() << " "
               << pose.position.z() << " " << pose.orientation.x() << " " << pose.orientation.y()
               << " " << pose.orientation.z() << " " << pose.orientation.w() << "\n";
        return static_cast<bool>(stream);
    }

    if (format == "euroc") {
        // Native EuRoC pose files use nanoseconds and scalar-first quaternions:
        // timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z.
        const slamforge::Pose pose = slamforge::Pose::FromSE3(Twc);
        const long long timestamp_ns = std::llround(timestamp * 1e9);
        stream << timestamp_ns << "," << pose.position.x() << "," << pose.position.y() << ","
               << pose.position.z() << "," << pose.orientation.w() << "," << pose.orientation.x()
               << "," << pose.orientation.y() << "," << pose.orientation.z() << "\n";
        return static_cast<bool>(stream);
    }

    if (format == "kitti") {
        const slamforge::Mat3 rotation = Twc.rotation();
        const slamforge::Vec3 translation = Twc.translation();
        for (int row = 0; row < 3; ++row) {
            stream << rotation(row, 0) << " " << rotation(row, 1) << " " << rotation(row, 2) << " "
                   << translation(row);
            stream << (row == 2 ? "\n" : " ");
        }
        return static_cast<bool>(stream);
    }

    return false;
}

struct TrackedPose {
    slamforge::FrameId frame_id;
    double timestamp = 0.0;
    slamforge::SE3 Tcw{slamforge::SE3::Identity()};
};

// =============================================================================
// SLAM pipeline — shared by `run` and `benchmark` subcommands
// =============================================================================

/// Run the full SLAM pipeline on an image directory or video file.
///
/// @returns EXIT_SUCCESS (0) or EXIT_FAILURE (1).
/// @param out_keyframes  If non-null, receives the final keyframe count.
/// @param out_mappoints  If non-null, receives the final map-point count.
static int RunSlam(const std::string& config_path, const std::string& input_path,
                   const std::string& output_path, bool verbose, double fps,
                   const std::string& output_format_override = "",
                   const std::string& timestamp_path = "", const std::string& map_output_path = "",
                   const std::string& gt_path = "", const std::string& gt_format = "tum",
                   int* out_keyframes = nullptr, int* out_mappoints = nullptr) {
    // ── Load configuration ──────────────────────────────────────────────────
    auto cfg_opt = slamforge::SystemConfig::LoadFromYAML(config_path);
    if (!cfg_opt) {
        std::cerr << "Error: failed to load config from " << config_path << "\n";
        return EXIT_FAILURE;
    }
    auto cfg = *cfg_opt;

    const std::string output_format =
        output_format_override.empty() ? cfg.output_format : output_format_override;
    if (!IsTrajectoryFormat(output_format)) {
        std::cerr << "Error: unsupported output format '" << output_format
                  << "' (expected tum, kitti, or euroc)\n";
        return EXIT_FAILURE;
    }
    if (fps <= 0.0) {
        std::cerr << "Error: --fps must be greater than zero\n";
        return EXIT_FAILURE;
    }

    // ORB, RANSAC and optical-flow reductions must produce the same map for
    // the same video on repeated batch runs.
    cv::setNumThreads(1);
    cv::setRNGSeed(0);

    // ── Build camera model ──────────────────────────────────────────────────
    slamforge::Camera::CameraParams cam_params{cfg.camera.fx,    cfg.camera.fy,    cfg.camera.cx,
                                               cfg.camera.cy,    cfg.camera.k1,    cfg.camera.k2,
                                               cfg.camera.p1,    cfg.camera.p2,    cfg.camera.k3,
                                               cfg.camera.width, cfg.camera.height};
    slamforge::Camera camera = slamforge::Camera::FromParams(cam_params);

    // ── Create tracker ──────────────────────────────────────────────────────
    slamforge::tracking::Tracker tracker(camera, cfg.tracking, cfg.orb);

    // ── Create local mapper ─────────────────────────────────────────────────
    slamforge::features::OrbExtractor::Options mapper_orb_opts;
    mapper_orb_opts.num_features = cfg.orb.num_features;
    mapper_orb_opts.scale_factor = static_cast<double>(cfg.orb.scale_factor);
    mapper_orb_opts.num_levels = cfg.orb.num_levels;
    mapper_orb_opts.ini_threshold = cfg.orb.ini_threshold;
    mapper_orb_opts.min_threshold = cfg.orb.min_threshold;
    mapper_orb_opts.patch_size = cfg.orb.patch_size;
    slamforge::features::OrbExtractor mapper_extractor(mapper_orb_opts);

    slamforge::mapping::LocalMapper local_mapper(tracker.GetMap(), camera, cfg.mapping,
                                                 mapper_extractor);
    tracker.SetLocalMapper(&local_mapper);

    // ── Create loop closer (optional) ──────────────────────────────────────
    // Tracker keeps a non-owning pointer and forwards every newly-created
    // keyframe to this worker.  Its lifetime therefore spans the full tracking
    // session and ends before the mapper/map are torn down.
    std::unique_ptr<slamforge::loop_closing::LoopClosing> loop_closing;
    if (cfg.loop_closing.enabled) {
        loop_closing = std::make_unique<slamforge::loop_closing::LoopClosing>(
            tracker.GetMap(), camera, cfg.loop_closing);

        if (!cfg.loop_closing.vocab_path.empty()) {
            if (!loop_closing->LoadVocabulary(cfg.loop_closing.vocab_path)) {
                std::cerr << "Error: failed to load loop-closing vocabulary: "
                          << cfg.loop_closing.vocab_path << "\n";
                return EXIT_FAILURE;
            }
        } else {
            std::cerr << "Warning: loop closing is enabled without vocab_path; "
                         "using the slower descriptor fallback.\n";
        }

        tracker.SetLoopClosing(loop_closing.get());
    }

    // ── Collect input ───────────────────────────────────────────────────────
    const bool is_video = IsVideoFile(input_path);
    std::vector<std::string> image_paths;
    std::vector<double> image_timestamps;
    int total_frames = 0;
    double effective_fps = fps;

    cv::VideoCapture video_cap;
    if (is_video) {
        video_cap.open(input_path);
        if (!video_cap.isOpened()) {
            std::cerr << "Error: cannot open video " << input_path << "\n";
            return EXIT_FAILURE;
        }
        total_frames = static_cast<int>(video_cap.get(cv::CAP_PROP_FRAME_COUNT));
        if (total_frames <= 0)
            total_frames = 1;  // Fallback for live streams
        const double detected_fps = video_cap.get(cv::CAP_PROP_FPS);
        if (std::isfinite(detected_fps) && detected_fps > 0.0) {
            effective_fps = detected_fps;
        }
    } else {
        std::string input_error;
        if (!GetImagePaths(input_path, image_paths, input_error)) {
            std::cerr << "Error: " << input_error << "\n";
            return EXIT_FAILURE;
        }
        if (image_paths.empty()) {
            std::cerr << "Error: no images found in " << input_path << "\n";
            return EXIT_FAILURE;
        }
        total_frames = static_cast<int>(image_paths.size());
        if (!timestamp_path.empty()) {
            std::string timestamp_error;
            if (!LoadImageTimestamps(timestamp_path, image_paths, image_timestamps,
                                     timestamp_error)) {
                std::cerr << "Error: " << timestamp_error << "\n";
                return EXIT_FAILURE;
            }
        }
    }
    if (is_video && !timestamp_path.empty()) {
        std::cerr << "Error: --timestamps is only supported for image directories\n";
        return EXIT_FAILURE;
    }

    // ── Print header ───────────────────────────────────────────────────────
    std::cout << "SLAMForge v" << slamforge::kVersionString << "\n";
    std::cout << "Config:  " << config_path << "\n";
    std::cout << "Input:   " << input_path << " (" << (is_video ? "video" : "images") << ", "
              << total_frames << " frames)\n";
    std::cout << "Output:  " << output_path << "\n";
    std::cout << "Format:  " << output_format << "\n";
    std::cout << "Camera:  " << cfg.camera.width << "x" << cfg.camera.height
              << "  fx=" << cfg.camera.fx << "\n";
    std::cout << "FPS:     " << effective_fps << "\n\n";
    if (!image_timestamps.empty()) {
        std::cout << "Timestamps: " << timestamp_path << "\n\n";
    }

    // ── Open trajectory file ───────────────────────────────────────────────
    std::ofstream traj_file(output_path);
    if (!traj_file.is_open()) {
        std::cerr << "Error: cannot open " << output_path << "\n";
        return EXIT_FAILURE;
    }
    if (output_format == "euroc") {
        traj_file << "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],"
                     "q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z []\n";
    }

    // ── Start local mapper ─────────────────────────────────────────────────
    local_mapper.Start();
    if (verbose)
        std::cout << "Local mapping thread started"
                  << (loop_closing ? "; loop closing queued for batch finalization" : "")
                  << "\n\n";

    const auto stop_background_workers = [&] {
        // Drain local mapping first.  Loop correction is finalized only after
        // tracking and mapping stop, so neither worker changes coordinate
        // systems underneath the live motion model.
        local_mapper.Stop();
        tracker.SetLocalMapper(nullptr);
        if (loop_closing) {
            // Run loop detection only after the complete local map is stable.
            // Batch results must not depend on mapper-vs-loop thread timing.
            loop_closing->Start();
            loop_closing->Stop();
            tracker.SetLoopClosing(nullptr);
        }
    };

    // ── Main tracking loop ─────────────────────────────────────────────────
    int frame_count = 0;
    int lost_count = 0;
    int initialization_count = 0;
    double timestamp = 0.0;
    const double dt = 1.0 / effective_fps;
    std::vector<TrackedPose> tracked_poses;
    tracked_poses.reserve(static_cast<size_t>(std::max(total_frames, 1)));
    int synchronized_keyframe_count = 0;

    for (int i = 0;; ++i) {
        cv::Mat image;

        if (is_video) {
            if (!video_cap.read(image) || image.empty())
                break;
        } else {
            if (i >= static_cast<int>(image_paths.size()))
                break;
            const size_t image_index = static_cast<size_t>(i);
            timestamp = image_timestamps.empty() ? static_cast<double>(i) * dt
                                                 : image_timestamps[image_index];
            image = cv::imread(image_paths[image_index], cv::IMREAD_COLOR);
            if (image.empty()) {
                if (verbose)
                    std::cerr << "Warning: cannot read " << image_paths[image_index] << "\n";
                continue;
            }
        }

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        auto pose_opt = tracker.Track(gray, timestamp);

        const int keyframe_count = tracker.NumKeyFrames();
        if (keyframe_count > synchronized_keyframe_count) {
            // Offline/desktop video processing values reproducibility and map
            // quality over latency: never outrun mapping at a KF boundary.
            local_mapper.WaitUntilIdle();
            synchronized_keyframe_count = keyframe_count;
            if (pose_opt) {
                pose_opt = tracker.SynchronizeCurrentPoseFromMap();
            }
        }

        if (pose_opt) {
            const slamforge::Frame* current_frame = tracker.CurrentFrame();
            if (current_frame) {
                tracked_poses.push_back(TrackedPose{current_frame->Id(), timestamp, *pose_opt});
            }
        } else {
            const auto state = tracker.State();
            if (state == slamforge::tracking::TrackingState::NOT_INITIALIZED ||
                state == slamforge::tracking::TrackingState::INITIALIZING) {
                ++initialization_count;
            } else {
                ++lost_count;
            }
        }

        // ── Progress indicator (stderr, overwrite line) ─────────────────
        if (frame_count % 10 == 0) {
            auto state = tracker.State();
            const char* state_str = "???";
            switch (state) {
                case slamforge::tracking::TrackingState::NOT_INITIALIZED:
                case slamforge::tracking::TrackingState::INITIALIZING:
                    state_str = "INIT";
                    break;
                case slamforge::tracking::TrackingState::OK:
                    state_str = "OK";
                    break;
                case slamforge::tracking::TrackingState::LOST:
                    state_str = "LOST";
                    break;
            }
            std::fprintf(stderr, "\rFrame %d/%d | state=%s | tracked=%d | kfs=%d", frame_count,
                         total_frames, state_str, tracker.NumTrackedPoints(),
                         tracker.NumKeyFrames());
        }

        if (is_video) {
            const double video_timestamp = video_cap.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
            timestamp = video_timestamp > 0.0 ? video_timestamp : timestamp + dt;
        }
        frame_count++;
    }

    std::fprintf(stderr, "\n");

    // ── Stop background workers ────────────────────────────────────────────
    stop_background_workers();
    if (verbose)
        std::cout << "Background mapping/loop-closing threads stopped\n";

    // A directory can contain image-named files that OpenCV cannot decode.
    // Do not report an empty processing run as a successful SLAM result.
    if (frame_count == 0) {
        std::cerr << "Error: no readable frames were processed from " << input_path << "\n";
        return EXIT_FAILURE;
    }

    // Trajectory poses are buffered so a loop closure can correct every past
    // frame consistently, rather than only keyframes that still live in Map.
    const auto loop_corrections =
        loop_closing ? loop_closing->Corrections()
                     : std::vector<slamforge::loop_closing::LoopCorrection>{};
    for (const TrackedPose& tracked_pose : tracked_poses) {
        slamforge::SE3 corrected_pose = tracked_pose.Tcw;
        for (const auto& loop_correction : loop_corrections) {
            const auto interpolated = slamforge::loop_closing::InterpolateLoopCorrection(
                loop_correction, tracked_pose.frame_id);
            corrected_pose =
                slamforge::loop_closing::ApplyLoopCorrectionToPose(corrected_pose, interpolated);
        }
        if (!WriteTrajectoryPose(traj_file, corrected_pose, tracked_pose.timestamp, output_format)) {
            std::cerr << "Error: failed to write trajectory output\n";
            return EXIT_FAILURE;
        }
    }
    traj_file.close();

    if (!map_output_path.empty()) {
        const auto export_result = slamforge::io::ExportMapAsPly(tracker.GetMap(), map_output_path);
        if (!export_result) {
            std::cerr << "Error: " << export_result.error << "\n";
            return EXIT_FAILURE;
        }
        if (verbose) {
            std::cout << "Map:     " << map_output_path << " (" << export_result.point_count
                      << " points)\n";
        }
    }

    // ── Collect final stats ─────────────────────────────────────────────────
    const int final_kfs = tracker.NumKeyFrames();
    const int final_mps = static_cast<int>(tracker.GetMap().MapPointCount());
    if (out_keyframes)
        *out_keyframes = final_kfs;
    if (out_mappoints)
        *out_mappoints = final_mps;

    // ── Summary ─────────────────────────────────────────────────────────────
    std::cout << "\n══════════════════════════════════════════════\n";
    std::cout << "  Tracking complete\n";
    std::cout << "  Frames:     " << frame_count << "\n";
    std::cout << "  Poses:      " << tracked_poses.size() << " ("
              << std::fixed << std::setprecision(1)
              << 100.0 * static_cast<double>(tracked_poses.size()) /
                     static_cast<double>(frame_count)
              << "%)\n";
    std::cout << "  Initialization: " << initialization_count << " frames\n";
    std::cout << "  Tracking lost: " << lost_count << " frames\n";
    std::cout << "  Keyframes:  " << final_kfs << "\n";
    std::cout << "  Map points: " << final_mps << "\n";
    if (loop_closing) {
        std::cout << "  Loops closed: " << loop_closing->NumLoopsClosed() << "\n";
    }
    std::cout << "  Output:     " << output_path << "\n";
    if (!map_output_path.empty()) {
        std::cout << "  Map:        " << map_output_path << "\n";
    }

    // ── Optional inline ATE ─────────────────────────────────────────────────
    if (!gt_path.empty()) {
        TrajectoryData est_traj = LoadTrajectory(output_path, output_format);
        TrajectoryData gt_traj = LoadTrajectory(gt_path, gt_format);
        if (!est_traj.empty() && !gt_traj.empty()) {
            double ate = ComputeATE(est_traj, gt_traj);
            double rpe = ComputeRPE(est_traj, gt_traj);
            std::cout << "  ATE RMSE:  " << ate << " m\n";
            std::cout << "  RPE RMSE:  " << rpe << " m\n";
        }
    }

    std::cout << "══════════════════════════════════════════════\n";
    return EXIT_SUCCESS;
}

// =============================================================================
// Subcommand: run
// =============================================================================

static int RunSubcommand(CLI::App* run_cmd) {
    // CLI11 stores references to bound values until CLI11_PARSE executes in
    // main().  These must therefore outlive this registration helper.
    static std::string config = "config/default.yaml";
    static std::string input;
    static std::string output = "trajectory.txt";
    static std::string map_output;
    static std::string output_format;
    static std::string timestamps;
    static bool verbose = false;
    static double fps = 30.0;

    run_cmd->add_option("--config", config, "YAML configuration file")
        ->required()
        ->check(CLI::ExistingFile);
    run_cmd
        ->add_option("--input", input, "Image directory or video file (.mp4/.avi/.mov/.mkv/.m4v)")
        ->required();
    run_cmd
        ->add_option("--output", output,
                     "Output trajectory (format from config unless --output-format is set)")
        ->default_val("trajectory.txt");
    run_cmd->add_option("--map-output", map_output, "Optional sparse map output in ASCII PLY");
    run_cmd
        ->add_option("--output-format", output_format,
                     "Output trajectory format override (tum|kitti|euroc)")
        ->check(CLI::IsMember({"tum", "kitti", "euroc"}));
    run_cmd
        ->add_option("--timestamps", timestamps,
                     "Timestamp file for an image sequence (TUM rgb.txt, EuRoC data.csv, or one "
                     "value per image)")
        ->check(CLI::ExistingFile);
    run_cmd->add_flag("--verbose", verbose, "Verbose per-frame output");
    run_cmd->add_option("--fps", fps, "Frame rate override for image sequences")->default_val(30.0);

    // Options are added; actual parsing happens in main() via CLI11_PARSE.
    // We return immediately — main() will call RunSlam after parsing.
    (void)run_cmd;  // suppress unused warning
    return 0;       // handled in main()
}

// =============================================================================
// Subcommand: eval
// =============================================================================

static int EvalSubcommand(CLI::App* eval_cmd) {
    static std::string estimated;
    static std::string groundtruth;
    static std::string format = "tum";
    static std::string estimated_format;
    static std::string groundtruth_format;
    static bool rpe = false;
    static bool plot = false;

    eval_cmd->add_option("--estimated", estimated, "Estimated trajectory file")
        ->required()
        ->check(CLI::ExistingFile);
    eval_cmd->add_option("--groundtruth", groundtruth, "Ground truth trajectory file")
        ->required()
        ->check(CLI::ExistingFile);
    eval_cmd
        ->add_option("--format", format, "Default format for both trajectories (tum|kitti|euroc)")
        ->default_val("tum")
        ->check(CLI::IsMember({"tum", "kitti", "euroc"}));
    eval_cmd
        ->add_option("--estimated-format", estimated_format,
                     "Estimated trajectory format override (tum|kitti|euroc)")
        ->check(CLI::IsMember({"tum", "kitti", "euroc"}));
    eval_cmd
        ->add_option("--groundtruth-format", groundtruth_format,
                     "Ground-truth trajectory format override (tum|kitti|euroc)")
        ->check(CLI::IsMember({"tum", "kitti", "euroc"}));
    eval_cmd->add_flag("--rpe", rpe, "Also compute Relative Pose Error");
    eval_cmd->add_flag("--plot", plot, "Plot trajectories (requires matplotlib)");

    (void)eval_cmd;  // options registered, handled in main()
    return 0;
}

// =============================================================================
// Subcommand: benchmark
// =============================================================================

static int BenchSubcommand(CLI::App* bench_cmd) {
    static std::string dataset_dir;
    static std::string config;
    static std::vector<std::string> sequences;
    static std::string output_dir = "results";

    bench_cmd->add_option("--dataset-dir", dataset_dir, "Dataset root directory")
        ->required()
        ->check(CLI::ExistingDirectory);
    bench_cmd->add_option("--config", config, "Configuration file")
        ->required()
        ->check(CLI::ExistingFile);
    bench_cmd->add_option("--sequences", sequences, "Sequence names (omit to discover all)");
    bench_cmd->add_option("--output-dir", output_dir, "Output directory for results")
        ->default_val("results");

    (void)bench_cmd;
    return 0;
}

// =============================================================================
// main
// =============================================================================

int main(int argc, char* argv[]) {
    executable_directory = ResolveExecutableDirectory(argc > 0 ? argv[0] : nullptr);
    CLI::App app{"SLAMForge — Industrial-Grade Monocular Visual SLAM"};
    app.set_version_flag("--version", slamforge::kVersionString);
    app.require_subcommand(1);

    // ── Set up subcommands ──────────────────────────────────────────────────
    auto* run = app.add_subcommand("run", "Run SLAM on images or video");
    auto* eval = app.add_subcommand("eval", "Evaluate trajectory against ground truth");
    auto* bench = app.add_subcommand("benchmark", "Batch-run SLAM on dataset sequences");

    // Register options on each subcommand
    RunSubcommand(run);
    EvalSubcommand(eval);
    BenchSubcommand(bench);

    CLI11_PARSE(app, argc, argv);

    // ── Dispatch: run ───────────────────────────────────────────────────────
    if (run->parsed()) {
        return RunSlam(run->get_option("--config")->as<std::string>(),
                       run->get_option("--input")->as<std::string>(),
                       run->get_option("--output")->as<std::string>(),
                       run->get_option("--verbose")->as<bool>(),
                       run->get_option("--fps")->as<double>(),
                       run->get_option("--output-format")->as<std::string>(),
                       run->get_option("--timestamps")->as<std::string>(),
                       run->get_option("--map-output")->as<std::string>());
    }

    // ── Dispatch: eval ──────────────────────────────────────────────────────
    if (eval->parsed()) {
        const std::string estimated = eval->get_option("--estimated")->as<std::string>();
        const std::string groundtruth = eval->get_option("--groundtruth")->as<std::string>();
        const std::string format = eval->get_option("--format")->as<std::string>();
        const std::string estimated_format =
            eval->get_option("--estimated-format")->as<std::string>().empty()
                ? format
                : eval->get_option("--estimated-format")->as<std::string>();
        const std::string groundtruth_format =
            eval->get_option("--groundtruth-format")->as<std::string>().empty()
                ? format
                : eval->get_option("--groundtruth-format")->as<std::string>();
        const bool do_rpe = eval->get_option("--rpe")->as<bool>();
        const bool do_plot = eval->get_option("--plot")->as<bool>();

        // Build ATE args
        std::vector<std::string> ate_args = {estimated,
                                             groundtruth,
                                             "--estimated-format",
                                             estimated_format,
                                             "--groundtruth-format",
                                             groundtruth_format};
        if (do_plot)
            ate_args.push_back("--plot");

        int ret = RunPythonTool("evaluate_ate.py", ate_args);

        // Optionally run RPE
        if (do_rpe) {
            std::vector<std::string> rpe_args = {estimated,
                                                 groundtruth,
                                                 "--estimated-format",
                                                 estimated_format,
                                                 "--groundtruth-format",
                                                 groundtruth_format};
            const int rpe_ret = RunPythonTool("evaluate_rpe.py", rpe_args);
            if (ret == EXIT_SUCCESS) {
                ret = rpe_ret;
            }
        }

        return ret;
    }

    // ── Dispatch: benchmark ─────────────────────────────────────────────────
    if (bench->parsed()) {
        const std::string dataset_dir = bench->get_option("--dataset-dir")->as<std::string>();
        const std::string config_path = bench->get_option("--config")->as<std::string>();
        const std::string output_dir = bench->get_option("--output-dir")->as<std::string>();

        auto cfg_opt = slamforge::SystemConfig::LoadFromYAML(config_path);
        if (!cfg_opt || !IsTrajectoryFormat(cfg_opt->output_format)) {
            std::cerr << "Error: unable to determine a supported output format from " << config_path
                      << "\n";
            return EXIT_FAILURE;
        }
        const std::string output_format = cfg_opt->output_format;

        // Collect sequence names
        std::vector<std::string> seqs;
        if (bench->get_option("--sequences")->empty()) {
            // Discover all subdirectories
            std::error_code filesystem_error;
            fs::directory_iterator iterator(dataset_dir, filesystem_error);
            const fs::directory_iterator end;
            if (filesystem_error) {
                std::cerr << "Error: cannot enumerate dataset directory " << dataset_dir << ": "
                          << filesystem_error.message() << "\n";
                return EXIT_FAILURE;
            }
            while (iterator != end) {
                const fs::directory_entry entry = *iterator;
                if (entry.is_directory(filesystem_error)) {
                    seqs.push_back(entry.path().filename().string());
                }
                if (filesystem_error) {
                    std::cerr << "Error: cannot inspect dataset entry: "
                              << filesystem_error.message() << "\n";
                    return EXIT_FAILURE;
                }
                iterator.increment(filesystem_error);
                if (filesystem_error) {
                    std::cerr << "Error: cannot enumerate dataset directory " << dataset_dir << ": "
                              << filesystem_error.message() << "\n";
                    return EXIT_FAILURE;
                }
            }
            std::sort(seqs.begin(), seqs.end());
        } else {
            seqs = bench->get_option("--sequences")->as<std::vector<std::string>>();
        }

        if (seqs.empty()) {
            std::cerr << "Error: no sequences found in " << dataset_dir << "\n";
            return EXIT_FAILURE;
        }

        std::error_code filesystem_error;
        fs::create_directories(output_dir, filesystem_error);
        if (filesystem_error) {
            std::cerr << "Error: cannot create output directory " << output_dir << ": "
                      << filesystem_error.message() << "\n";
            return EXIT_FAILURE;
        }

        // Storage for summary table
        struct SeqResult {
            std::string name;
            int frames = 0;
            double ate_rmse = -1.0;
            double rpe_rmse = -1.0;
            int keyframes = 0;
            int mappoints = 0;
            bool success = false;
        };
        std::vector<SeqResult> results;

        std::cout << "\n══════════════════════════════════════════════\n";
        std::cout << "  SLAMForge Benchmark\n";
        std::cout << "  Dataset: " << dataset_dir << "\n";
        std::cout << "  Sequences: " << seqs.size() << "\n";
        std::cout << "══════════════════════════════════════════════\n\n";

        for (const auto& seq : seqs) {
            SeqResult r;
            r.name = seq;

            fs::path seq_path = fs::path(dataset_dir) / seq;
            fs::path image_path = seq_path / "image_0";
            fs::path out_traj = fs::path(output_dir) / (seq + "_traj.txt");

            // Skip if image directory missing
            if (!fs::is_directory(image_path, filesystem_error) || filesystem_error) {
                std::cerr << "Warning: no image_0/ in " << seq << ", skipping\n";
                filesystem_error.clear();
                r.success = false;
                results.push_back(r);
                continue;
            }

            // Locate ground truth
            std::string gt_path;
            std::string gt_format = "tum";
            if (fs::exists(seq_path / "poses.txt", filesystem_error) && !filesystem_error) {
                gt_path = (seq_path / "poses.txt").string();
                gt_format = "kitti";
            } else if (!filesystem_error &&
                       fs::exists(seq_path / "groundtruth.txt", filesystem_error) &&
                       !filesystem_error) {
                gt_path = (seq_path / "groundtruth.txt").string();
                gt_format = "tum";
            }
            filesystem_error.clear();

            std::cout << "[" << seq << "] Running SLAM...\n";

            int run_kfs = 0, run_mps = 0;
            int ret =
                RunSlam(config_path, image_path.string(), out_traj.string(), /*verbose=*/false,
                        /*fps=*/30.0, /*output_format_override=*/"", /*timestamp_path=*/"",
                        /*map_output_path=*/"", gt_path, gt_format, &run_kfs, &run_mps);

            r.success = (ret == EXIT_SUCCESS);
            r.keyframes = run_kfs;
            r.mappoints = run_mps;

            if (r.success) {
                // Count frames from output trajectory
                TrajectoryData out_data = LoadTrajectory(out_traj.string(), output_format);
                r.frames = static_cast<int>(out_data.size());

                // Compute ATE / RPE if ground truth available
                if (!gt_path.empty()) {
                    TrajectoryData gt_data = LoadTrajectory(gt_path, gt_format);
                    if (!gt_data.empty()) {
                        r.ate_rmse = ComputeATE(out_data, gt_data);
                        r.rpe_rmse = ComputeRPE(out_data, gt_data);
                    }
                }
            }

            results.push_back(r);
            std::cout << "\n";
        }

        // ── Summary table ───────────────────────────────────────────────────
        std::cout << "══════════════════════════════════════════════════════════════════\n";
        std::cout << "  Benchmark Summary\n";
        std::cout << "══════════════════════════════════════════════════════════════════\n";
        std::printf("  %-12s %8s %10s %10s %8s %8s\n", "Sequence", "Frames", "ATE RMSE", "RPE RMSE",
                    "KFs", "MPs");
        std::cout << "  " << std::string(12 + 8 + 10 + 10 + 8 + 8 + 5, '-') << "\n";

        for (const auto& r : results) {
            if (r.success) {
                std::printf("  %-12s %8d %10.4f %10.4f %8d %8d\n", r.name.c_str(), r.frames,
                            r.ate_rmse, r.rpe_rmse, r.keyframes, r.mappoints);
            } else {
                std::printf("  %-12s %8s %10s %10s %8s %8s\n", r.name.c_str(), "-", "-", "-", "-",
                            "-");
            }
        }
        std::cout << "══════════════════════════════════════════════════════════════════\n";

        return EXIT_SUCCESS;
    }

    return EXIT_SUCCESS;
}
