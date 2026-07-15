// =============================================================================
// LoopDetector — BoW-based loop candidate detection
// =============================================================================
// Queries a BoW database to find keyframes that visually overlap with the
// current frame. Implements temporal consistency checks (consecutive
// detections) following ORB-SLAM3's pattern.

#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "litevo/core/types.h"
#include "litevo/loop_closing/vocabulary.h"

namespace litevo {

class KeyFrame;
class Map;

namespace loop_closing {

/// @brief Configuration for the LoopDetector.
struct LoopDetectorConfig {
    double min_score = 0.3;          ///< Minimum BoW similarity to consider
    int min_consecutive = 3;         ///< Consecutive detections to accept a loop
    int max_recent_kfs = 20;         ///< Ignore the N most recent keyframes
    double min_inlier_ratio = 0.25;  ///< Minimum match ratio for fallback scoring
};

/// @brief Detects loop closure candidates from keyframe BoW vectors.
///
/// Maintains an inverted index mapping vocabulary words to keyframes,
/// and a database of BoW vectors for fast similarity queries.
class LoopDetector {
public:
    explicit LoopDetector(const LoopDetectorConfig& config = {});

    // ── Database management ────────────────────────────────────────────────

    /// @brief Add a keyframe to the BoW database.
    void Insert(std::shared_ptr<KeyFrame> kf);

    /// @brief Query for the best loop candidate for a keyframe.
    std::shared_ptr<KeyFrame> Detect(std::shared_ptr<KeyFrame> current_kf, const Map& map);

    // ── Vocabulary ─────────────────────────────────────────────────────────

    const Vocabulary& GetVocabulary() const { return vocabulary_; }
    bool LoadVocabulary(const std::string& path);

    int DatabaseSize() const { return database_size_; }

    /// @brief Detector settings in effect for this worker.
    const LoopDetectorConfig& Config() const noexcept { return config_; }

private:
    double ScoreBetween(std::shared_ptr<KeyFrame> kf1, std::shared_ptr<KeyFrame> kf2);
    std::shared_ptr<KeyFrame> DetectWithFallback(std::shared_ptr<KeyFrame> current_kf,
                                                 const Map& map);

    LoopDetectorConfig config_;
    Vocabulary vocabulary_;

    // BoW vectors and word weights stored per keyframe
    std::unordered_map<FrameId, std::vector<float>, FrameIdHash> bow_vectors_;
    std::unordered_map<FrameId, std::vector<std::pair<int, float>>, FrameIdHash> word_weights_;

    int database_size_ = 0;

    // Consecutive detection tracking
    FrameId last_candidate_id_{0};
    int consecutive_count_ = 0;
};

}  // namespace loop_closing
}  // namespace litevo
