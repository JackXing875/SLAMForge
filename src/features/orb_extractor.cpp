// =============================================================================
// SLAMForge ORB extractor — implementation
// =============================================================================

#include "slamforge/features/orb_extractor.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace slamforge::features {

OrbExtractor::OrbExtractor() : opts_{} {
    ComputePyramidParameters();
    orb_ = cv::ORB::create(opts_.num_features, static_cast<float>(opts_.scale_factor),
                           opts_.num_levels, opts_.ini_threshold, 0, 2, cv::ORB::HARRIS_SCORE,
                           opts_.patch_size, opts_.min_threshold);
}

OrbExtractor::OrbExtractor(const Options& opts) : opts_(opts) {
    ComputePyramidParameters();

    orb_ = cv::ORB::create(opts_.num_features, static_cast<float>(opts_.scale_factor),
                           opts_.num_levels, opts_.ini_threshold,
                           0,  // firstLevel
                           2,  // WTA_K (2 = Hamming distance)
                           cv::ORB::HARRIS_SCORE, opts_.patch_size,
                           opts_.min_threshold  // fastThreshold (minimum)
    );
}

void OrbExtractor::ComputePyramidParameters() {
    const auto num_levels = static_cast<size_t>(opts_.num_levels);
    scale_factors_.resize(num_levels);
    inv_scale_factors_.resize(num_levels);
    inv_scale_factors_sq_.resize(num_levels);
    level_sigmas_.resize(num_levels);

    scale_factors_[0] = 1.0f;
    inv_scale_factors_[0] = 1.0f;
    inv_scale_factors_sq_[0] = 1.0f;
    level_sigmas_[0] = 1.0f;

    for (int level = 1; level < opts_.num_levels; ++level) {
        const auto index = static_cast<size_t>(level);
        const auto previous = static_cast<size_t>(level - 1);
        scale_factors_[index] = scale_factors_[previous] * static_cast<float>(opts_.scale_factor);
        inv_scale_factors_[index] = 1.0f / scale_factors_[index];
        inv_scale_factors_sq_[index] = inv_scale_factors_[index] * inv_scale_factors_[index];
        level_sigmas_[index] = scale_factors_[index];
    }
}

int OrbExtractor::Extract(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                          cv::Mat& descriptors, const cv::Mat& mask) const {
    if (image.empty() || image.channels() != 1) {
        return 0;
    }

    orb_->detectAndCompute(image, mask, keypoints, descriptors);

    // Cap at requested number
    if (static_cast<int>(keypoints.size()) > opts_.num_features) {
        // Sort by response (highest first) and keep top N
        std::partial_sort(
            keypoints.begin(), keypoints.begin() + opts_.num_features, keypoints.end(),
            [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });
        keypoints.resize(static_cast<size_t>(opts_.num_features));

        // Trim descriptors to match
        if (!descriptors.empty()) {
            descriptors = descriptors.rowRange(0, opts_.num_features);
        }
    }

    return static_cast<int>(keypoints.size());
}

int OrbExtractor::ExtractUniform(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                                 cv::Mat& descriptors, const cv::Mat& mask) const {
    if (image.empty() || image.channels() != 1) {
        return 0;
    }

    // Step 1: Extract features with a lower threshold to get more candidates
    cv::Ptr<cv::ORB> orb_low =
        cv::ORB::create(opts_.num_features * 2,  // more candidates
                        static_cast<float>(opts_.scale_factor), opts_.num_levels,
                        opts_.min_threshold,  // use lower threshold for more candidates
                        0, 2, cv::ORB::HARRIS_SCORE, opts_.patch_size, opts_.min_threshold);

    std::vector<cv::KeyPoint> all_keypoints;
    cv::Mat all_descriptors;
    orb_low->detectAndCompute(image, mask, all_keypoints, all_descriptors);

    if (all_keypoints.empty()) {
        keypoints.clear();
        descriptors = cv::Mat();
        return 0;
    }

    // Step 2: Group keypoints by octave
    const auto num_levels = static_cast<size_t>(opts_.num_levels);
    std::vector<std::vector<cv::KeyPoint>> kps_by_level(num_levels);
    for (const auto& kp : all_keypoints) {
        if (kp.octave >= 0 && kp.octave < opts_.num_levels) {
            kps_by_level[static_cast<size_t>(kp.octave)].push_back(kp);
        }
    }

    // Step 3: Compute target count per level based on scale factors
    std::vector<int> target_per_level(num_levels, 0);
    float total_area = 0.0f;
    for (int level = 0; level < opts_.num_levels; ++level) {
        total_area += 1.0f / scale_factors_[static_cast<size_t>(level)];
    }
    int assigned = 0;
    for (int level = 0; level < opts_.num_levels; ++level) {
        const auto index = static_cast<size_t>(level);
        target_per_level[index] = static_cast<int>(static_cast<float>(opts_.num_features) *
                                                   (1.0f / scale_factors_[index]) / total_area);
        assigned += target_per_level[index];
    }
    // Distribute remainder
    target_per_level[0] += (opts_.num_features - assigned);

    // Step 4: Distribute per level using quadtree
    keypoints.clear();
    std::vector<int> selected_indices;

    for (int level = 0; level < opts_.num_levels; ++level) {
        const auto index = static_cast<size_t>(level);
        const auto& level_kps = kps_by_level[index];
        if (level_kps.empty())
            continue;

        int target = std::min(target_per_level[index], static_cast<int>(level_kps.size()));

        if (target <= 0)
            continue;

        // Sort by response for this level
        auto sorted = level_kps;
        std::partial_sort(
            sorted.begin(), sorted.begin() + target, sorted.end(),
            [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });

        for (int i = 0; i < target; ++i) {
            keypoints.push_back(sorted[static_cast<size_t>(i)]);
        }
    }

    // Step 5: Recompute descriptors for selected keypoints
    if (!keypoints.empty()) {
        orb_->compute(image, keypoints, descriptors);
    }

    return static_cast<int>(keypoints.size());
}

std::vector<cv::KeyPoint> OrbExtractor::DistributeOctTree(
    const std::vector<cv::KeyPoint>& keypoints, int /*min_x*/, int /*max_x*/, int /*min_y*/,
    int /*max_y*/, int num_features, int /*level*/) const {
    if (static_cast<int>(keypoints.size()) <= num_features) {
        return keypoints;
    }

    // Simplified quadtree: sort by response and take best
    auto result = keypoints;
    std::partial_sort(
        result.begin(), result.begin() + num_features, result.end(),
        [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });
    result.resize(static_cast<size_t>(num_features));
    return result;
}

}  // namespace slamforge::features
