// =============================================================================
// LiteVO CLI — command-line monocular visual SLAM
// =============================================================================
// Subcommand-based CLI using CLI11:
//   litevo run     — Run SLAM on images or video
//   litevo eval    — Evaluate trajectory against ground truth
//   litevo bench   — Batch-run SLAM on dataset sequences
// =============================================================================

#include <Eigen/SVD>

#include <CLI/CLI.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "litevo/litevo.h"

namespace fs = std::filesystem;

// =============================================================================
// Image / video input helpers
// =============================================================================

namespace {

/// Check whether a file path has a recognized video extension.
bool IsVideoFile(const std::string& path) {
    std::string ext = fs::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext == ".mp4" || ext == ".avi" || ext == ".mov" || ext == ".mkv" || ext == ".m4v";
}

/// Collect and sort image paths from a directory.
std::vector<std::string> GetImagePaths(const std::string& dir_path) {
    std::vector<std::string> paths;
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (!entry.is_regular_file())
            continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".ppm") {
            paths.push_back(entry.path().string());
        }
    }
    std::sort(paths.begin(), paths.end());
    return paths;
}

}  // anonymous namespace

// =============================================================================
// Python tool runner (for eval subcommand)
// =============================================================================

/// Shell out to a Python evaluation script in tools/.
/// @returns the exit code from std::system (0 on success).
static int RunPythonTool(const std::string& tool_name, const std::vector<std::string>& args) {
    fs::path tool_path = fs::path("tools") / tool_name;

    std::string cmd = "python3 " + tool_path.string();
    for (const auto& arg : args) {
        // Quote arguments containing spaces
        if (arg.find(' ') != std::string::npos) {
            cmd += " \"" + arg + "\"";
        } else {
            cmd += " " + arg;
        }
    }

    std::cout << "  Running: " << cmd << "\n";
    return std::system(cmd.c_str());
}

// =============================================================================
// Trajectory I/O and inline ATE / RPE computation
// =============================================================================

/// Lightweight trajectory representation (translations only).
struct TrajectoryData {
    std::vector<litevo::Vec3> positions;

    bool empty() const { return positions.empty(); }
    size_t size() const { return positions.size(); }
};

/// Load a trajectory file.  Supported formats: "tum", "kitti".
static TrajectoryData LoadTrajectory(const std::string& path, const std::string& format) {
    TrajectoryData data;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open trajectory file: " << path << "\n";
        return data;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#')
            continue;

        std::istringstream iss(line);
        double tx = 0.0, ty = 0.0, tz = 0.0;

        if (format == "kitti") {
            // KITTI pose: 3x4 matrix flattened row-major
            //   r11 r12 r13 tx  r21 r22 r23 ty  r31 r32 r33 tz
            double val = 0.0;
            for (int i = 0; i < 12; ++i) {
                if (!(iss >> val))
                    break;
                if (i == 3)
                    tx = val;
                else if (i == 7)
                    ty = val;
                else if (i == 11)
                    tz = val;
            }
        } else {
            // TUM:  timestamp tx ty tz qx qy qz qw
            double timestamp = 0.0;
            iss >> timestamp >> tx >> ty >> tz;
        }

        data.positions.push_back(litevo::Vec3(tx, ty, tz));
    }

    return data;
}

/// Umeyama alignment: find rotation R and translation t that minimise
///   sum_i || R * src_i + t - dst_i ||^2
/// Scale is fixed at 1 (appropriate for trajectory evaluation).
/// @param[out] aligned  src points transformed by the optimal (R, t).
/// @returns 1.0 (scale).
static double AlignUmeyama(const std::vector<litevo::Vec3>& src,
                           const std::vector<litevo::Vec3>& dst,
                           std::vector<litevo::Vec3>& aligned) {
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
        P.row(i) = src[i].transpose();
        Q.row(i) = dst[i].transpose();
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

    // Translation
    Eigen::Vector3d t = mean_q.transpose() - R * mean_p.transpose();

    // Apply
    for (size_t i = 0; i < N; ++i) {
        aligned[i] = R * src[i] + t;
    }

    return 1.0;  // scale fixed
}

/// Compute Absolute Trajectory Error RMSE (after Umeyama alignment).
static double ComputeATE(const TrajectoryData& est, const TrajectoryData& gt) {
    const size_t N = std::min(est.size(), gt.size());
    if (N == 0)
        return -1.0;

    std::vector<litevo::Vec3> aligned;
    AlignUmeyama(est.positions, gt.positions, aligned);

    double sum_sq = 0.0;
    for (size_t i = 0; i < N; ++i) {
        sum_sq += (aligned[i] - gt.positions[i]).squaredNorm();
    }
    return std::sqrt(sum_sq / static_cast<double>(N));
}

/// Compute translational Relative Pose Error RMSE (after Umeyama alignment).
/// Uses frame delta = 1.
static double ComputeRPE(const TrajectoryData& est, const TrajectoryData& gt, int delta = 1) {
    const size_t N = std::min(est.size(), gt.size());
    if (N <= static_cast<size_t>(delta) || N == 0)
        return -1.0;

    // Align the full trajectory first
    std::vector<litevo::Vec3> est_aligned;
    AlignUmeyama(est.positions, gt.positions, est_aligned);

    double sum_sq = 0.0;
    size_t count = 0;
    for (size_t i = 0; i + static_cast<size_t>(delta) < N; ++i) {
        litevo::Vec3 est_delta = est_aligned[i + delta] - est_aligned[i];
        litevo::Vec3 gt_delta = gt.positions[i + delta] - gt.positions[i];
        sum_sq += (est_delta - gt_delta).squaredNorm();
        ++count;
    }
    if (count == 0)
        return -1.0;
    return std::sqrt(sum_sq / static_cast<double>(count));
}

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
                   const std::string& gt_path = "", const std::string& format = "tum",
                   int* out_keyframes = nullptr, int* out_mappoints = nullptr) {
    // ── Load configuration ──────────────────────────────────────────────────
    auto cfg_opt = litevo::SystemConfig::LoadFromYAML(config_path);
    if (!cfg_opt) {
        std::cerr << "Error: failed to load config from " << config_path << "\n";
        return EXIT_FAILURE;
    }
    auto cfg = *cfg_opt;

    // ── Build camera model ──────────────────────────────────────────────────
    litevo::Camera::CameraParams cam_params{cfg.camera.fx,    cfg.camera.fy,    cfg.camera.cx,
                                            cfg.camera.cy,    cfg.camera.k1,    cfg.camera.k2,
                                            cfg.camera.p1,    cfg.camera.p2,    cfg.camera.k3,
                                            cfg.camera.width, cfg.camera.height};
    litevo::Camera camera = litevo::Camera::FromParams(cam_params);

    // ── Create tracker ──────────────────────────────────────────────────────
    litevo::tracking::Tracker tracker(camera, cfg.tracking, cfg.orb);

    // ── Create local mapper ─────────────────────────────────────────────────
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

    // ── Collect input ───────────────────────────────────────────────────────
    const bool is_video = IsVideoFile(input_path);
    std::vector<std::string> image_paths;
    int total_frames = 0;

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
    } else {
        if (!fs::is_directory(input_path)) {
            std::cerr << "Error: input must be a directory or video file\n";
            return EXIT_FAILURE;
        }
        image_paths = GetImagePaths(input_path);
        if (image_paths.empty()) {
            std::cerr << "Error: no images found in " << input_path << "\n";
            return EXIT_FAILURE;
        }
        total_frames = static_cast<int>(image_paths.size());
    }

    // ── Print header ───────────────────────────────────────────────────────
    std::cout << "LiteVO v" << litevo::kVersionString << "\n";
    std::cout << "Config:  " << config_path << "\n";
    std::cout << "Input:   " << input_path << " (" << (is_video ? "video" : "images") << ", "
              << total_frames << " frames)\n";
    std::cout << "Output:  " << output_path << "\n";
    std::cout << "Camera:  " << cfg.camera.width << "x" << cfg.camera.height
              << "  fx=" << cfg.camera.fx << "\n";
    std::cout << "FPS:     " << fps << "\n\n";

    // ── Open trajectory file ───────────────────────────────────────────────
    std::ofstream traj_file(output_path);
    if (!traj_file.is_open()) {
        std::cerr << "Error: cannot open " << output_path << "\n";
        return EXIT_FAILURE;
    }

    // ── Start local mapper ─────────────────────────────────────────────────
    local_mapper.Start();
    if (verbose)
        std::cout << "Local mapping thread started\n\n";

    // ── Main tracking loop ─────────────────────────────────────────────────
    int frame_count = 0;
    int lost_count = 0;
    double timestamp = 0.0;
    const double dt = 1.0 / fps;

    for (int i = 0;; ++i) {
        cv::Mat image;

        if (is_video) {
            if (!video_cap.read(image) || image.empty())
                break;
        } else {
            if (i >= static_cast<int>(image_paths.size()))
                break;
            image = cv::imread(image_paths[i], cv::IMREAD_COLOR);
            if (image.empty()) {
                if (verbose)
                    std::cerr << "Warning: cannot read " << image_paths[i] << "\n";
                continue;
            }
        }

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        auto pose_opt = tracker.Track(gray, timestamp);

        if (pose_opt) {
            // TUM format:  timestamp tx ty tz qx qy qz qw  (Twc = world-from-camera)
            litevo::SE3 Twc = pose_opt->inverse();
            litevo::Pose p = litevo::Pose::FromSE3(Twc);

            traj_file << timestamp << " " << p.position.x() << " " << p.position.y() << " "
                      << p.position.z() << " " << p.orientation.x() << " " << p.orientation.y()
                      << " " << p.orientation.z() << " " << p.orientation.w() << "\n";
        } else {
            lost_count++;
        }

        // ── Progress indicator (stderr, overwrite line) ─────────────────
        if (frame_count % 10 == 0) {
            auto state = tracker.State();
            const char* state_str = "???";
            switch (state) {
                case litevo::tracking::TrackingState::NOT_INITIALIZED:
                case litevo::tracking::TrackingState::INITIALIZING:
                    state_str = "INIT";
                    break;
                case litevo::tracking::TrackingState::OK:
                    state_str = "OK";
                    break;
                case litevo::tracking::TrackingState::LOST:
                    state_str = "LOST";
                    break;
            }
            std::fprintf(stderr, "\rFrame %d/%d | state=%s | tracked=%d | kfs=%d", frame_count,
                         total_frames, state_str, tracker.NumTrackedPoints(),
                         tracker.NumKeyFrames());
        }

        timestamp += dt;
        frame_count++;
    }

    std::fprintf(stderr, "\n");
    traj_file.close();

    // ── Stop local mapper ──────────────────────────────────────────────────
    local_mapper.Stop();
    if (verbose)
        std::cout << "Local mapping thread stopped\n";

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
    std::cout << "  Lost:       " << lost_count << "\n";
    std::cout << "  Keyframes:  " << final_kfs << "\n";
    std::cout << "  Map points: " << final_mps << "\n";
    std::cout << "  Output:     " << output_path << "\n";

    // ── Optional inline ATE ─────────────────────────────────────────────────
    if (!gt_path.empty()) {
        TrajectoryData est_traj = LoadTrajectory(output_path, "tum");
        TrajectoryData gt_traj = LoadTrajectory(gt_path, format);
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
    std::string config = "config/default.yaml";
    std::string input;
    std::string output = "trajectory.txt";
    bool verbose = false;
    double fps = 30.0;

    run_cmd->add_option("--config", config, "YAML configuration file")
        ->required()
        ->check(CLI::ExistingFile);
    run_cmd
        ->add_option("--input", input, "Image directory or video file (.mp4/.avi/.mov/.mkv/.m4v)")
        ->required();
    run_cmd->add_option("--output", output, "Output trajectory (TUM format)");
    run_cmd->add_flag("--verbose", verbose, "Verbose per-frame output");
    run_cmd->add_option("--fps", fps, "Frame rate override for image sequences");

    // Options are added; actual parsing happens in main() via CLI11_PARSE.
    // We return immediately — main() will call RunSlam after parsing.
    (void)run_cmd;  // suppress unused warning
    return 0;       // handled in main()
}

// =============================================================================
// Subcommand: eval
// =============================================================================

static int EvalSubcommand(CLI::App* eval_cmd) {
    std::string estimated;
    std::string groundtruth;
    std::string format = "tum";
    bool rpe = false;
    bool plot = false;

    eval_cmd->add_option("--estimated", estimated, "Estimated trajectory file")
        ->required()
        ->check(CLI::ExistingFile);
    eval_cmd->add_option("--groundtruth", groundtruth, "Ground truth trajectory file")
        ->required()
        ->check(CLI::ExistingFile);
    eval_cmd->add_option("--format", format, "Trajectory format (tum|kitti|euroc)")
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
    std::string dataset_dir;
    std::string config;
    std::vector<std::string> sequences;
    std::string output_dir = "results";

    bench_cmd->add_option("--dataset-dir", dataset_dir, "Dataset root directory")
        ->required()
        ->check(CLI::ExistingDirectory);
    bench_cmd->add_option("--config", config, "Configuration file")
        ->required()
        ->check(CLI::ExistingFile);
    bench_cmd->add_option("--sequences", sequences, "Sequence names (omit to discover all)");
    bench_cmd->add_option("--output-dir", output_dir, "Output directory for results");

    (void)bench_cmd;
    return 0;
}

// =============================================================================
// main
// =============================================================================

int main(int argc, char* argv[]) {
    CLI::App app{"LiteVO — Industrial-Grade Monocular Visual SLAM"};
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
                       run->get_option("--fps")->as<double>());
    }

    // ── Dispatch: eval ──────────────────────────────────────────────────────
    if (eval->parsed()) {
        const std::string estimated = eval->get_option("--estimated")->as<std::string>();
        const std::string groundtruth = eval->get_option("--groundtruth")->as<std::string>();
        const std::string format = eval->get_option("--format")->as<std::string>();
        const bool do_rpe = eval->get_option("--rpe")->as<bool>();
        const bool do_plot = eval->get_option("--plot")->as<bool>();

        // Build ATE args
        std::vector<std::string> ate_args = {estimated, groundtruth, "--format", format};
        if (do_plot)
            ate_args.push_back("--plot");

        int ret = RunPythonTool("evaluate_ate.py", ate_args);

        // Optionally run RPE
        if (do_rpe) {
            std::vector<std::string> rpe_args = {estimated, groundtruth, "--format", format};
            ret |= RunPythonTool("evaluate_rpe.py", rpe_args);
        }

        return ret;
    }

    // ── Dispatch: benchmark ─────────────────────────────────────────────────
    if (bench->parsed()) {
        const std::string dataset_dir = bench->get_option("--dataset-dir")->as<std::string>();
        const std::string config_path = bench->get_option("--config")->as<std::string>();
        const std::string output_dir = bench->get_option("--output-dir")->as<std::string>();

        // Collect sequence names
        std::vector<std::string> seqs;
        if (bench->get_option("--sequences")->empty()) {
            // Discover all subdirectories
            for (const auto& entry : fs::directory_iterator(dataset_dir)) {
                if (entry.is_directory())
                    seqs.push_back(entry.path().filename().string());
            }
            std::sort(seqs.begin(), seqs.end());
        } else {
            seqs = bench->get_option("--sequences")->as<std::vector<std::string>>();
        }

        if (seqs.empty()) {
            std::cerr << "Error: no sequences found in " << dataset_dir << "\n";
            return EXIT_FAILURE;
        }

        fs::create_directories(output_dir);

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
        std::cout << "  LiteVO Benchmark\n";
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
            if (!fs::is_directory(image_path)) {
                std::cerr << "Warning: no image_0/ in " << seq << ", skipping\n";
                r.success = false;
                results.push_back(r);
                continue;
            }

            // Locate ground truth
            std::string gt_path;
            std::string gt_format = "tum";
            if (fs::exists(seq_path / "poses.txt")) {
                gt_path = (seq_path / "poses.txt").string();
                gt_format = "kitti";
            } else if (fs::exists(seq_path / "groundtruth.txt")) {
                gt_path = (seq_path / "groundtruth.txt").string();
                gt_format = "tum";
            }

            std::cout << "[" << seq << "] Running SLAM...\n";

            int run_kfs = 0, run_mps = 0;
            int ret =
                RunSlam(config_path, image_path.string(), out_traj.string(), /*verbose=*/false,
                        /*fps=*/30.0, gt_path, gt_format, &run_kfs, &run_mps);

            r.success = (ret == EXIT_SUCCESS);
            r.keyframes = run_kfs;
            r.mappoints = run_mps;

            if (r.success) {
                // Count frames from output trajectory
                TrajectoryData out_data = LoadTrajectory(out_traj.string(), "tum");
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
