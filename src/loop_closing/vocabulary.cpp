// =============================================================================
// Vocabulary implementation — FBOW wrapper with descriptor fallback
// =============================================================================

#include "slamforge/loop_closing/vocabulary.h"

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <algorithm>
#include <cmath>

#ifdef SLAMFORGE_HAS_FBOW
#include <fbow/fbow.h>
#endif

namespace slamforge::loop_closing {

#ifdef SLAMFORGE_HAS_FBOW
namespace {
// Global FBOW vocabulary instance (lazy-loaded, shared)
std::unique_ptr<fbow::Vocabulary> g_fbow_vocab;
}  // namespace
#endif

bool Vocabulary::Load(const std::string& path) {
#ifdef SLAMFORGE_HAS_FBOW
    try {
        auto vocab = std::make_unique<fbow::Vocabulary>();
        vocab->readFromFile(path);
        if (!vocab->isValid())
            return false;
        g_fbow_vocab = std::move(vocab);
        num_words_ = static_cast<int>(g_fbow_vocab->size());
        is_loaded_ = true;
        return true;
    } catch (const std::exception&) {
        return false;
    }
#else
    (void)path;
    return false;
#endif
}

void Vocabulary::Transform(const cv::Mat& descriptors, std::vector<float>& bow,
                           std::vector<std::pair<int, float>>& word_weights) const {
    bow.clear();
    word_weights.clear();

    if (!is_loaded_ || descriptors.empty())
        return;

#ifdef SLAMFORGE_HAS_FBOW
    if (!g_fbow_vocab || !g_fbow_vocab->isValid())
        return;

    fbow::fBow fbow_vec = g_fbow_vocab->transform(descriptors);

    int n_words = static_cast<int>(g_fbow_vocab->size());
    bow.resize(static_cast<size_t>(n_words), 0.0f);

    for (const auto& kv : fbow_vec) {
        int word_id = static_cast<int>(kv.first);
        float weight = static_cast<float>(kv.second);
        if (word_id >= 0 && word_id < n_words) {
            bow[static_cast<size_t>(word_id)] = weight;
            word_weights.emplace_back(word_id, weight);
        }
    }
#else
    (void)descriptors;
#endif
}

double Vocabulary::Score(const std::vector<float>& bow1, const std::vector<float>& bow2) const {
    if (bow1.empty() || bow2.empty())
        return 0.0;

    double score = 0.0;
    size_t n = std::min(bow1.size(), bow2.size());
    for (size_t i = 0; i < n; ++i) {
        score += static_cast<double>(std::min(bow1[i], bow2[i]));
    }
    return score;
}

// ── Fallback: descriptor-based similarity ─────────────────────────────────────

double ComputeDescriptorSimilarity(const cv::Mat& desc1, const cv::Mat& desc2,
                                   float ratio_threshold) {
    if (desc1.empty() || desc2.empty())
        return 0.0;

    cv::BFMatcher matcher(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(desc1, desc2, knn_matches, 2);

    int good = 0;
    for (const auto& knn : knn_matches) {
        if (knn.size() >= 2) {
            if (knn[0].distance < ratio_threshold * knn[1].distance) {
                good++;
            }
        } else if (knn.size() == 1) {
            good++;
        }
    }

    if (desc1.rows == 0)
        return 0.0;
    return static_cast<double>(good) / static_cast<double>(desc1.rows);
}

}  // namespace slamforge::loop_closing
