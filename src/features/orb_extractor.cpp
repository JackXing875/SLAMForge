// =============================================================================
// LiteVO ORB extractor — implementation
// =============================================================================

#include "litevo/features/orb_extractor.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace litevo::features {

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
    scale_factors_.resize(opts_.num_levels);
    inv_scale_factors_.resize(opts_.num_levels);
    inv_scale_factors_sq_.resize(opts_.num_levels);
    level_sigmas_.resize(opts_.num_levels);

    scale_factors_[0] = 1.0f;
    inv_scale_factors_[0] = 1.0f;
    inv_scale_factors_sq_[0] = 1.0f;
    level_sigmas_[0] = 1.0f;

    for (int level = 1; level < opts_.num_levels; ++level) {
        scale_factors_[level] = scale_factors_[level - 1] * static_cast<float>(opts_.scale_factor);
        inv_scale_factors_[level] = 1.0f / scale_factors_[level];
        inv_scale_factors_sq_[level] = inv_scale_factors_[level] * inv_scale_factors_[level];
        level_sigmas_[level] = scale_factors_[level];
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
        keypoints.resize(opts_.num_features);

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
        cv::ORB::create(static_cast<int>(opts_.num_features * 2),  // more candidates
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
    std::vector<std::vector<cv::KeyPoint>> kps_by_level(opts_.num_levels);
    for (const auto& kp : all_keypoints) {
        kps_by_level[kp.octave].push_back(kp);
    }

    // Step 3: Compute target count per level based on scale factors
    std::vector<int> target_per_level(opts_.num_levels, 0);
    float total_area = 0.0f;
    for (int level = 0; level < opts_.num_levels; ++level) {
        total_area += 1.0f / scale_factors_[level];
    }
    int assigned = 0;
    for (int level = 0; level < opts_.num_levels; ++level) {
        target_per_level[level] =
            static_cast<int>(opts_.num_features * (1.0f / scale_factors_[level]) / total_area);
        assigned += target_per_level[level];
    }
    // Distribute remainder
    target_per_level[0] += (opts_.num_features - assigned);

    // Step 4: Distribute per level using quadtree
    keypoints.clear();
    std::vector<int> selected_indices;

    for (int level = 0; level < opts_.num_levels; ++level) {
        const auto& level_kps = kps_by_level[level];
        if (level_kps.empty())
            continue;

        int target = std::min(target_per_level[level], static_cast<int>(level_kps.size()));

        if (target <= 0)
            continue;

        // Sort by response for this level
        auto sorted = level_kps;
        std::partial_sort(
            sorted.begin(), sorted.begin() + target, sorted.end(),
            [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });

        for (int i = 0; i < target; ++i) {
            keypoints.push_back(sorted[i]);
        }
    }

    // Step 5: Recompute descriptors for selected keypoints
    if (!keypoints.empty()) {
        orb_->compute(image, keypoints, descriptors);
    }

    return static_cast<int>(keypoints.size());
}

std::vector<cv::KeyPoint> OrbExtractor::DistributeOctTree(
    const std::vector<cv::KeyPoint>& keypoints, int min_x, int max_x, int min_y, int max_y,
    int num_features, int /*level*/) const {
    if (static_cast<int>(keypoints.size()) <= num_features) {
        return keypoints;
    }

    // Simplified quadtree: sort by response and take best
    auto result = keypoints;
    std::partial_sort(
        result.begin(), result.begin() + num_features, result.end(),
        [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });
    result.resize(num_features);
    return result;
}

}  // namespace litevo::features
