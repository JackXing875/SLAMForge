// =============================================================================
// Vocabulary — BoW vocabulary wrapper for loop detection
// =============================================================================
// Wraps FBOW (Fast Bag-of-Words) for ORB descriptor quantization and
// keyframe similarity scoring. When FBOW is unavailable, provides a
// brute-force fallback for descriptor matching.

#pragma once

#include <opencv2/core/mat.hpp>

#include <string>
#include <utility>
#include <vector>

namespace litevo {

class KeyFrame;

namespace loop_closing {

/// @brief Configuration for the Vocabulary.
struct VocabularyConfig {
    std::string vocab_path;  ///< Path to pre-trained vocabulary file
    int branching = 10;      ///< K-means branching factor (FBOW)
    int depth = 6;           ///< Tree depth (FBOW)
};

/// @brief Bag-of-words vocabulary for keyframe similarity scoring.
///
/// Uses FBOW when available (LITEVO_HAS_FBOW). Falls back to descriptor
/// distance-based similarity when FBOW is not available.
class Vocabulary {
public:
    Vocabulary() = default;

    /// @brief Load a pre-trained vocabulary from file.
    bool Load(const std::string& path);

    /// @brief Check if a vocabulary is loaded and ready.
    bool IsLoaded() const { return is_loaded_; }

    /// @brief Transform a set of descriptors into a BoW vector.
    /// @param descriptors  ORB descriptor matrix (N x 32, CV_8UC1).
    /// @param bow          Output BoW vector (vocabulary-size floats).
    /// @param word_weights Output (word_id, weight) pairs for matched words.
    void Transform(const cv::Mat& descriptors, std::vector<float>& bow,
                   std::vector<std::pair<int, float>>& word_weights) const;

    /// @brief Compute L1 similarity score between two BoW vectors.
    double Score(const std::vector<float>& bow1, const std::vector<float>& bow2) const;

    /// @brief Size of the vocabulary (number of words), or 0 if not loaded.
    int Size() const { return num_words_; }

private:
    bool is_loaded_ = false;
    int num_words_ = 0;
};

/// @brief A simple BoW-like similarity using descriptor matching.
///
/// Used as a fallback when FBOW is unavailable. Computes similarity
/// by matching descriptors between two keyframes and scoring based
/// on the ratio of matched descriptors.
double ComputeDescriptorSimilarity(const cv::Mat& desc1, const cv::Mat& desc2,
                                   float ratio_threshold = 0.7f);

}  // namespace loop_closing
}  // namespace litevo
