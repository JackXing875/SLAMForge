// =============================================================================
// SLAMForge offline dense surface reconstruction
// =============================================================================

#include "slamforge/mapping/dense_mapper.h"

#include <Eigen/Core>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

#ifdef SLAMFORGE_HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

#include "slamforge/core/camera.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"

namespace slamforge::mapping {
namespace {

double Quantile(std::vector<double> values, double fraction) {
    if (values.empty()) {
        return 0.0;
    }
    const size_t index = static_cast<size_t>(std::clamp(fraction, 0.0, 1.0) *
                                             static_cast<double>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index),
                     values.end());
    return values[index];
}

struct LinearFit {
    double scale = 0.0;
    double offset = 0.0;
    bool valid = false;
};

LinearFit FitRobustLine(const std::vector<std::pair<double, double>>& samples) {
    if (samples.size() < 3) {
        return {};
    }

    std::vector<double> weights(samples.size(), 1.0);
    LinearFit result;
    for (int iteration = 0; iteration < 8; ++iteration) {
        double sum_weight = 0.0;
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_xx = 0.0;
        double sum_xy = 0.0;
        for (size_t index = 0; index < samples.size(); ++index) {
            const auto [x, y] = samples[index];
            const double weight = weights[index];
            sum_weight += weight;
            sum_x += weight * x;
            sum_y += weight * y;
            sum_xx += weight * x * x;
            sum_xy += weight * x * y;
        }
        const double determinant = sum_weight * sum_xx - sum_x * sum_x;
        if (!std::isfinite(determinant) || std::abs(determinant) < 1e-12) {
            return {};
        }
        result.scale = (sum_weight * sum_xy - sum_x * sum_y) / determinant;
        result.offset = (sum_y - result.scale * sum_x) / sum_weight;
        if (!std::isfinite(result.scale) || !std::isfinite(result.offset)) {
            return {};
        }

        std::vector<double> residuals;
        residuals.reserve(samples.size());
        for (const auto& [x, y] : samples) {
            residuals.push_back(std::abs(result.scale * x + result.offset - y));
        }
        const double median_residual = Quantile(residuals, 0.5);
        const double huber_delta = std::max(1e-9, median_residual * 2.5);
        for (size_t index = 0; index < residuals.size(); ++index) {
            weights[index] = residuals[index] <= huber_delta
                                 ? 1.0
                                 : huber_delta / std::max(residuals[index], 1e-12);
        }
    }
    result.valid = true;
    return result;
}

DepthCalibration EvaluateHypothesis(const std::vector<DepthAnchor>& anchors,
                                    DepthCalibration::Mode mode) {
    std::vector<std::pair<double, double>> line_samples;
    line_samples.reserve(anchors.size());
    for (const DepthAnchor& anchor : anchors) {
        if (!std::isfinite(anchor.prediction) || !std::isfinite(anchor.camera_depth) ||
            anchor.camera_depth <= 1e-6) {
            continue;
        }
        const double target = mode == DepthCalibration::Mode::kInverseDepth
                                  ? 1.0 / anchor.camera_depth
                                  : anchor.camera_depth;
        line_samples.emplace_back(anchor.prediction, target);
    }
    const LinearFit fit = FitRobustLine(line_samples);
    if (!fit.valid) {
        return {};
    }

    DepthCalibration calibration;
    calibration.mode = mode;
    calibration.scale = fit.scale;
    calibration.offset = fit.offset;

    std::vector<double> depths;
    std::vector<double> errors;
    depths.reserve(anchors.size());
    errors.reserve(anchors.size());
    for (const DepthAnchor& anchor : anchors) {
        const double estimated = calibration.Evaluate(anchor.prediction);
        if (!std::isfinite(estimated) || estimated <= 1e-6) {
            continue;
        }
        depths.push_back(anchor.camera_depth);
        errors.push_back(std::abs(estimated - anchor.camera_depth) / anchor.camera_depth);
    }
    if (errors.size() < 3) {
        return {};
    }
    calibration.median_relative_error = Quantile(errors, 0.5);
    calibration.minimum_depth = std::max(1e-5, Quantile(depths, 0.05) * 0.45);
    calibration.maximum_depth = Quantile(depths, 0.95) * 2.2;
    calibration.inlier_count = static_cast<int>(
        std::count_if(errors.begin(), errors.end(), [](double error) { return error < 0.35; }));
    return calibration;
}

#ifdef SLAMFORGE_HAS_ONNXRUNTIME

cv::Mat ColorForKeyFrame(const KeyFrame& keyframe, int output_width) {
    cv::Mat color;
    if (!keyframe.ColorImage().empty()) {
        color = keyframe.ColorImage();
    } else {
        cv::cvtColor(keyframe.Image(), color, cv::COLOR_GRAY2BGR);
    }
    const int width = std::max(32, output_width);
    const int height = std::max(
        24, static_cast<int>(std::lround(static_cast<double>(color.rows) * width / color.cols)));
    cv::Mat resized;
    cv::resize(color, resized, cv::Size(width, height), 0.0, 0.0, cv::INTER_AREA);
    return resized;
}

std::vector<std::shared_ptr<KeyFrame>> SelectKeyFrames(Map& map, int maximum) {
    std::vector<std::shared_ptr<KeyFrame>> candidates;
    for (const auto& keyframe : map.GetAllKeyFrames()) {
        if (keyframe && !keyframe->IsBad() && keyframe->Pose().matrix().allFinite() &&
            !keyframe->Image().empty()) {
            candidates.push_back(keyframe);
        }
    }
    if (candidates.size() <= static_cast<size_t>(std::max(2, maximum))) {
        return candidates;
    }

    const int count = std::max(2, maximum);
    std::vector<std::shared_ptr<KeyFrame>> selected;
    selected.reserve(static_cast<size_t>(count));
    size_t previous = candidates.size();
    for (int index = 0; index < count; ++index) {
        const size_t candidate_index = static_cast<size_t>(
            std::llround(static_cast<double>(index) * static_cast<double>(candidates.size() - 1) /
                         static_cast<double>(count - 1)));
        if (candidate_index != previous) {
            selected.push_back(candidates[candidate_index]);
            previous = candidate_index;
        }
    }
    return selected;
}

std::vector<DepthAnchor> BuildAnchors(const KeyFrame& keyframe, Map& map,
                                      const cv::Mat& prediction) {
    std::vector<DepthAnchor> anchors;
    const auto& keypoints = keyframe.KeyPoints();
    anchors.reserve(static_cast<size_t>(keyframe.NumMapPoints()));
    for (int feature = 0; feature < keyframe.NumKeyPoints(); ++feature) {
        auto map_point = map.GetMapPoint(keyframe.MapPointIdAt(feature));
        if (!map_point || map_point->IsBad() || map_point->Observations() < 2 ||
            map_point->GetFoundRatio() < 0.2F) {
            continue;
        }
        const Vec3 camera_point = keyframe.Pose() * map_point->Position();
        if (!camera_point.allFinite() || camera_point.z() <= 1e-6 ||
            feature >= static_cast<int>(keypoints.size())) {
            continue;
        }
        const cv::Point2f pixel = keypoints[static_cast<size_t>(feature)].pt;
        const int x =
            std::clamp(static_cast<int>(std::lround(static_cast<double>(pixel.x) * prediction.cols /
                                                    keyframe.GetCamera().width())),
                       0, prediction.cols - 1);
        const int y =
            std::clamp(static_cast<int>(std::lround(static_cast<double>(pixel.y) * prediction.rows /
                                                    keyframe.GetCamera().height())),
                       0, prediction.rows - 1);
        const float model_value = prediction.at<float>(y, x);
        if (std::isfinite(model_value)) {
            anchors.push_back({static_cast<double>(model_value), camera_point.z()});
        }
    }
    return anchors;
}

struct DenseFrame {
    std::shared_ptr<KeyFrame> keyframe;
    cv::Mat color;
    cv::Mat prediction;
    cv::Mat depth;
    DepthCalibration calibration;
};

#ifdef SLAMFORGE_HAS_ONNXRUNTIME
class DepthNetwork {
public:
    explicit DepthNetwork(const std::filesystem::path& model_path)
        : environment_(ORT_LOGGING_LEVEL_WARNING, "slamforge-dense"), session_(nullptr) {
        Ort::SessionOptions options;
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        const unsigned int hardware_threads = std::thread::hardware_concurrency();
        options.SetIntraOpNumThreads(static_cast<int>(std::clamp(hardware_threads, 1U, 4U)));
#ifdef _WIN32
        session_ = Ort::Session(environment_, model_path.wstring().c_str(), options);
#else
        session_ = Ort::Session(environment_, model_path.c_str(), options);
#endif
    }

    cv::Mat Infer(const cv::Mat& bgr, const cv::Size& output_size) {
        constexpr int model_size = 518;
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        cv::resize(rgb, rgb, cv::Size(model_size, model_size), 0.0, 0.0, cv::INTER_CUBIC);
        rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

        std::vector<float> input(3 * model_size * model_size);
        constexpr std::array<float, 3> mean{0.485F, 0.456F, 0.406F};
        constexpr std::array<float, 3> deviation{0.229F, 0.224F, 0.225F};
        for (int y = 0; y < model_size; ++y) {
            const auto* row = rgb.ptr<cv::Vec3f>(y);
            for (int x = 0; x < model_size; ++x) {
                for (int channel = 0; channel < 3; ++channel) {
                    input[static_cast<size_t>(channel * model_size * model_size + y * model_size +
                                              x)] =
                        (row[x][channel] - mean[static_cast<size_t>(channel)]) /
                        deviation[static_cast<size_t>(channel)];
                }
            }
        }

        std::array<int64_t, 4> shape{1, 3, model_size, model_size};
        auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto tensor = Ort::Value::CreateTensor<float>(memory, input.data(), input.size(),
                                                      shape.data(), shape.size());
        constexpr std::array<const char*, 1> input_names{"image"};
        constexpr std::array<const char*, 1> output_names{"depth"};
        auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names.data(), &tensor, 1,
                                    output_names.data(), 1);
        const float* values = outputs.front().GetTensorData<float>();
        const size_t value_count = outputs.front().GetTensorTypeAndShapeInfo().GetElementCount();
        if (value_count != static_cast<size_t>(model_size * model_size)) {
            return {};
        }
        cv::Mat model_output(model_size, model_size, CV_32F, const_cast<float*>(values));
        cv::Mat result;
        cv::resize(model_output, result, output_size, 0.0, 0.0, cv::INTER_CUBIC);
        return result.clone();
    }

private:
    Ort::Env environment_;
    Ort::Session session_;
};
#endif

cv::Mat CalibratedDepth(const cv::Mat& prediction, const DepthCalibration& calibration) {
    cv::Mat depth(prediction.size(), CV_32F, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    for (int y = 0; y < prediction.rows; ++y) {
        const float* prediction_row = prediction.ptr<float>(y);
        float* depth_row = depth.ptr<float>(y);
        for (int x = 0; x < prediction.cols; ++x) {
            const double value = calibration.Evaluate(prediction_row[x]);
            if (std::isfinite(value) && value >= calibration.minimum_depth &&
                value <= calibration.maximum_depth) {
                depth_row[x] = static_cast<float>(value);
            }
        }
    }
    return depth;
}

std::optional<bool> ConsistentInFrame(const Vec3& world_point, const DenseFrame& other,
                                      const Camera& camera, double tolerance) {
    const Vec3 camera_point = other.keyframe->Pose() * world_point;
    if (!camera_point.allFinite() || camera_point.z() <= 1e-6) {
        return std::nullopt;
    }
    const Vec2 pixel = camera.Project(camera_point);
    if (!camera.IsInImage(pixel, 2)) {
        return std::nullopt;
    }
    const int x =
        std::clamp(static_cast<int>(std::lround(pixel.x() * other.depth.cols / camera.width())), 0,
                   other.depth.cols - 1);
    const int y =
        std::clamp(static_cast<int>(std::lround(pixel.y() * other.depth.rows / camera.height())), 0,
                   other.depth.rows - 1);
    const float reference_depth = other.depth.at<float>(y, x);
    if (!std::isfinite(reference_depth) || reference_depth <= 1e-6F) {
        return std::nullopt;
    }
    return std::abs(camera_point.z() - static_cast<double>(reference_depth)) /
               std::max(camera_point.z(), static_cast<double>(reference_depth)) <=
           tolerance;
}

struct VoxelKey {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    bool operator==(const VoxelKey&) const = default;
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& key) const noexcept {
        size_t result = std::hash<int64_t>{}(key.x);
        result ^= std::hash<int64_t>{}(key.y) + 0x9e3779b9U + (result << 6U) + (result >> 2U);
        result ^= std::hash<int64_t>{}(key.z) + 0x9e3779b9U + (result << 6U) + (result >> 2U);
        return result;
    }
};

struct VoxelValue {
    Vec3 position_sum = Vec3::Zero();
    std::array<uint64_t, 3> color_sum{0, 0, 0};
    int samples = 0;
    int views = 0;
    int last_view = -1;
};

struct OutputPoint {
    Vec3 position = Vec3::Zero();
    std::array<int, 3> rgb{0, 0, 0};
    float confidence = 0.0F;
    VoxelKey key;
};

bool WriteDensePly(const std::filesystem::path& output_path, const std::vector<OutputPoint>& points,
                   std::string& error) {
    std::ofstream output(output_path, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        error = "cannot open dense map output '" + output_path.string() + "'";
        return false;
    }
    output << "ply\n"
           << "format ascii 1.0\n"
           << "comment Generated by SLAMForge dense reconstruction\n"
           << "element vertex " << points.size() << "\n"
           << "property double x\nproperty double y\nproperty double z\n"
           << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
           << "property float confidence\nend_header\n";
    output << std::setprecision(9);
    for (const OutputPoint& point : points) {
        output << point.position.x() << ' ' << point.position.y() << ' ' << point.position.z()
               << ' ' << point.rgb[0] << ' ' << point.rgb[1] << ' ' << point.rgb[2] << ' '
               << point.confidence << '\n';
    }
    if (!output) {
        error = "failed while writing dense map output '" + output_path.string() + "'";
        return false;
    }
    return true;
}

#endif  // SLAMFORGE_HAS_ONNXRUNTIME

}  // namespace

double DepthCalibration::Evaluate(double prediction) const noexcept {
    if (!valid() || !std::isfinite(prediction)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double mapped = scale * prediction + offset;
    if (!std::isfinite(mapped) || mapped <= 1e-9) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return mode == Mode::kInverseDepth ? 1.0 / mapped : mapped;
}

DepthCalibration CalibrateRelativeDepth(const std::vector<DepthAnchor>& anchors,
                                        int minimum_samples, double maximum_median_error) {
    if (anchors.size() < static_cast<size_t>(std::max(3, minimum_samples))) {
        return {};
    }
    DepthCalibration inverse = EvaluateHypothesis(anchors, DepthCalibration::Mode::kInverseDepth);
    DepthCalibration direct = EvaluateHypothesis(anchors, DepthCalibration::Mode::kDirectDepth);
    DepthCalibration best =
        inverse.median_relative_error <= direct.median_relative_error ? inverse : direct;
    const int minimum_inliers = std::max(6, minimum_samples / 2);
    if (!best.valid() || best.median_relative_error > maximum_median_error ||
        best.inlier_count < minimum_inliers || !std::isfinite(best.minimum_depth) ||
        !std::isfinite(best.maximum_depth) || best.maximum_depth <= best.minimum_depth) {
        return {};
    }
    return best;
}

DenseMapper::DenseMapper(const Camera& camera, DenseMappingConfig config)
    : camera_(camera), config_(config) {}

bool DenseMapper::RuntimeAvailable() noexcept {
#ifdef SLAMFORGE_HAS_ONNXRUNTIME
    return true;
#else
    return false;
#endif
}

DenseReconstructionResult DenseMapper::Reconstruct(Map& map,
                                                   const std::filesystem::path& model_path,
                                                   const std::filesystem::path& output_path,
                                                   const ProgressCallback& progress) const {
    DenseReconstructionResult result;
#ifndef SLAMFORGE_HAS_ONNXRUNTIME
    (void)map;
    (void)model_path;
    (void)output_path;
    (void)progress;
    result.error = "this SLAMForge build does not include ONNX Runtime dense reconstruction";
    return result;
#else
    if (model_path.empty() || !std::filesystem::is_regular_file(model_path)) {
        result.error = "dense depth model not found at '" + model_path.string() + "'";
        return result;
    }
    if (output_path.empty()) {
        result.error = "dense map output path is empty";
        return result;
    }

    std::vector<DenseFrame> frames;
    {
        auto graph_lock = map.AcquireGraphLock();
        const auto keyframes = SelectKeyFrames(map, config_.max_keyframes);
        result.selected_keyframes = static_cast<int>(keyframes.size());
        frames.reserve(keyframes.size());
        for (const auto& keyframe : keyframes) {
            DenseFrame frame;
            frame.keyframe = keyframe;
            frame.color = ColorForKeyFrame(*keyframe, config_.output_width);
            frames.push_back(std::move(frame));
        }
    }
    if (frames.size() < 2) {
        result.error = "dense reconstruction needs at least two valid keyframes";
        return result;
    }

    try {
        DepthNetwork network(model_path);
        std::vector<double> scene_depths;
        const int total_progress = static_cast<int>(frames.size() * 2);
        for (size_t index = 0; index < frames.size(); ++index) {
            DenseFrame& frame = frames[index];
            frame.prediction = network.Infer(frame.color, frame.color.size());
            if (frame.prediction.empty()) {
                continue;
            }
            const auto anchors = BuildAnchors(*frame.keyframe, map, frame.prediction);
            frame.calibration = CalibrateRelativeDepth(anchors, config_.min_sparse_samples,
                                                       config_.max_calibration_error);
            if (frame.calibration.valid()) {
                frame.depth = CalibratedDepth(frame.prediction, frame.calibration);
                ++result.calibrated_keyframes;
                for (const DepthAnchor& anchor : anchors) {
                    if (std::isfinite(anchor.camera_depth) && anchor.camera_depth > 0.0) {
                        scene_depths.push_back(anchor.camera_depth);
                    }
                }
            }
            if (progress) {
                progress(static_cast<int>(index) + 1, total_progress);
            }
        }

        if (result.calibrated_keyframes < 2 || scene_depths.empty()) {
            result.error = "too few keyframes could be calibrated against sparse SLAM depth";
            return result;
        }
        const double median_scene_depth = Quantile(scene_depths, 0.5);
        result.voxel_size = std::max(1e-5, median_scene_depth * config_.voxel_size_ratio);

        std::unordered_map<VoxelKey, VoxelValue, VoxelKeyHash> voxels;
        const int pixel_stride = std::max(1, config_.pixel_stride);
        for (size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
            const DenseFrame& frame = frames[frame_index];
            if (frame.depth.empty()) {
                if (progress) {
                    progress(static_cast<int>(frames.size() + frame_index + 1), total_progress);
                }
                continue;
            }
            const SE3 camera_to_world = frame.keyframe->Pose().inverse();
            for (int y = pixel_stride; y + pixel_stride < frame.depth.rows; y += pixel_stride) {
                const float* depth_row = frame.depth.ptr<float>(y);
                const auto* color_row = frame.color.ptr<cv::Vec3b>(y);
                for (int x = pixel_stride; x + pixel_stride < frame.depth.cols; x += pixel_stride) {
                    const float depth = depth_row[x];
                    if (!std::isfinite(depth) || depth <= 1e-6F) {
                        continue;
                    }
                    const float horizontal_left = frame.depth.at<float>(y, x - pixel_stride);
                    const float horizontal_right = frame.depth.at<float>(y, x + pixel_stride);
                    const float vertical_top = frame.depth.at<float>(y - pixel_stride, x);
                    const float vertical_bottom = frame.depth.at<float>(y + pixel_stride, x);
                    if (!std::isfinite(horizontal_left) || !std::isfinite(horizontal_right) ||
                        !std::isfinite(vertical_top) || !std::isfinite(vertical_bottom)) {
                        continue;
                    }
                    const float discontinuity = std::max(
                        {std::abs(horizontal_left - depth), std::abs(horizontal_right - depth),
                         std::abs(vertical_top - depth), std::abs(vertical_bottom - depth)});
                    if (discontinuity / depth > 0.28F) {
                        continue;
                    }

                    const Vec2 source_pixel(
                        (static_cast<double>(x) + 0.5) * camera_.width() / frame.depth.cols,
                        (static_cast<double>(y) + 0.5) * camera_.height() / frame.depth.rows);
                    const Vec3 ray = camera_.Unproject(source_pixel);
                    if (!ray.allFinite() || ray.z() <= 1e-9) {
                        continue;
                    }
                    const Vec3 world_point =
                        camera_to_world * (ray * (static_cast<double>(depth) / ray.z()));
                    if (!world_point.allFinite()) {
                        continue;
                    }

                    int checked_views = 0;
                    int supporting_views = 0;
                    for (const int offset : {-1, 1}) {
                        const int neighbor_index = static_cast<int>(frame_index) + offset;
                        if (neighbor_index < 0 ||
                            neighbor_index >= static_cast<int>(frames.size()) ||
                            frames[static_cast<size_t>(neighbor_index)].depth.empty()) {
                            continue;
                        }
                        const auto consistent = ConsistentInFrame(
                            world_point, frames[static_cast<size_t>(neighbor_index)], camera_,
                            config_.consistency_tolerance);
                        if (!consistent.has_value()) {
                            continue;
                        }
                        ++checked_views;
                        if (*consistent) {
                            ++supporting_views;
                        }
                    }
                    // A point is dense-map evidence only when another
                    // calibrated keyframe observes compatible depth there.
                    // Single-view predictions remain useful for inference but
                    // must not become unsupported building surfaces.
                    if (checked_views == 0 || supporting_views == 0) {
                        continue;
                    }

                    const VoxelKey key{
                        static_cast<int64_t>(std::floor(world_point.x() / result.voxel_size)),
                        static_cast<int64_t>(std::floor(world_point.y() / result.voxel_size)),
                        static_cast<int64_t>(std::floor(world_point.z() / result.voxel_size))};
                    VoxelValue& voxel = voxels[key];
                    voxel.position_sum += world_point;
                    const cv::Vec3b bgr = color_row[x];
                    voxel.color_sum[0] += bgr[2];
                    voxel.color_sum[1] += bgr[1];
                    voxel.color_sum[2] += bgr[0];
                    ++voxel.samples;
                    if (voxel.last_view != static_cast<int>(frame_index)) {
                        voxel.last_view = static_cast<int>(frame_index);
                        ++voxel.views;
                    }
                }
            }
            if (progress) {
                progress(static_cast<int>(frames.size() + frame_index + 1), total_progress);
            }
        }

        std::vector<OutputPoint> points;
        points.reserve(voxels.size());
        for (const auto& [key, voxel] : voxels) {
            if (voxel.samples <= 0) {
                continue;
            }
            OutputPoint point;
            point.position = voxel.position_sum / static_cast<double>(voxel.samples);
            const auto sample_count = static_cast<uint64_t>(voxel.samples);
            point.rgb = {static_cast<int>(voxel.color_sum[0] / sample_count),
                         static_cast<int>(voxel.color_sum[1] / sample_count),
                         static_cast<int>(voxel.color_sum[2] / sample_count)};
            point.confidence = std::min(1.0F, static_cast<float>(voxel.views) / 3.0F);
            point.key = key;
            points.push_back(point);
        }
        std::sort(points.begin(), points.end(), [](const OutputPoint& lhs, const OutputPoint& rhs) {
            if (lhs.key.x != rhs.key.x) {
                return lhs.key.x < rhs.key.x;
            }
            if (lhs.key.y != rhs.key.y) {
                return lhs.key.y < rhs.key.y;
            }
            return lhs.key.z < rhs.key.z;
        });
        const size_t maximum_points =
            static_cast<size_t>(std::max(1000, config_.max_output_points));
        if (points.size() > maximum_points) {
            std::vector<OutputPoint> sampled;
            sampled.reserve(maximum_points);
            for (size_t index = 0; index < maximum_points; ++index) {
                const size_t source = index * points.size() / maximum_points;
                sampled.push_back(points[source]);
            }
            points = std::move(sampled);
        }
        if (points.size() < 1000) {
            result.error = "multi-view filtering produced too few dense surface points";
            return result;
        }
        if (!WriteDensePly(output_path, points, result.error)) {
            return result;
        }
        result.success = true;
        result.point_count = points.size();
        return result;
    } catch (const Ort::Exception& exception) {
        result.error = std::string("ONNX Runtime depth inference failed: ") + exception.what();
        return result;
    } catch (const cv::Exception& exception) {
        result.error = std::string("OpenCV dense reconstruction failed: ") + exception.what();
        return result;
    } catch (const std::exception& exception) {
        result.error = std::string("dense reconstruction failed: ") + exception.what();
        return result;
    }
#endif
}

}  // namespace slamforge::mapping
