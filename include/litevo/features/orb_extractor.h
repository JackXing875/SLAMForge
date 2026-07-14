// =============================================================================
// LiteVO ORB feature extractor — multi-scale oriented FAST + BRIEF
// =============================================================================

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <vector>

namespace litevo::features {

/// @brief ORB feature extraction with quadtree distribution.
///
/// Extracts ORB features at multiple pyramid scales and distributes them
/// uniformly using quadtree-based non-maximum suppression.
/// Wraps OpenCV's ORB with uniform distribution guarantees.
class OrbExtractor {
public:
    /// @brief Extractor configuration.
    struct Options {
        int num_features = 1200;    ///< Target number of features
        double scale_factor = 1.2;  ///< Scale factor between pyramid levels
        int num_levels = 8;         ///< Number of pyramid levels
        int ini_threshold = 20;     ///< FAST initial threshold
        int min_threshold = 7;      ///< FAST minimum threshold
        int patch_size = 31;        ///< ORB descriptor patch size
    };

    OrbExtractor();
    explicit OrbExtractor(const Options& opts);

    /// @brief Extract ORB features from an image.
    ///
    /// @param image Input grayscale image (CV_8UC1)
    /// @param keypoints Output keypoints with octave information
    /// @param descriptors Output ORB descriptors (CV_8UC1, 32 bytes per feature)
    /// @param mask Optional mask (255 = extract, 0 = skip)
    /// @return Number of features extracted
    int Extract(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                const cv::Mat& mask = cv::Mat()) const;

    /// @brief Extract features and distribute uniformly using quadtree.
    ///
    /// Unlike plain Extract(), this method ensures features are
    /// well-distributed across the image by using quadtree-based
    /// non-maximum suppression.
    int ExtractUniform(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                       cv::Mat& descriptors, const cv::Mat& mask = cv::Mat()) const;

    /// @brief Get image pyramid scale factors.
    [[nodiscard]] const std::vector<float>& ScaleFactors() const { return scale_factors_; }

    /// @brief Get inverse scale factors.
    [[nodiscard]] const std::vector<float>& InvScaleFactors() const { return inv_scale_factors_; }

    /// @brief Get sigma values for each level.
    [[nodiscard]] const std::vector<float>& LevelSigmas() const { return level_sigmas_; }

    /// @brief Get the complete scale factor (products).
    [[nodiscard]] const std::vector<float>& InvScaleFactorsSquared() const {
        return inv_scale_factors_sq_;
    }

private:
    Options opts_;
    cv::Ptr<cv::ORB> orb_;

    std::vector<float> scale_factors_;
    std::vector<float> inv_scale_factors_;
    std::vector<float> inv_scale_factors_sq_;
    std::vector<float> level_sigmas_;

    void ComputePyramidParameters();

    /// @brief Distribute keypoints uniformly using quadtree.
    void DistributeQuadtree(const std::vector<cv::KeyPoint>& keypoints, int min_x, int max_x,
                            int min_y, int max_y, int num_features, int level) const;

    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& keypoints,
                                                int min_x, int max_x, int min_y, int max_y,
                                                int num_features, int level) const;
};

}  // namespace litevo::features
