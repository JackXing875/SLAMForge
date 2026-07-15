// =============================================================================
// LoopDetector implementation
// =============================================================================

#include "slamforge/loop_closing/detector.h"

#include <opencv2/features2d.hpp>

#include <algorithm>
#include <set>

#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"

namespace slamforge::loop_closing {

LoopDetector::LoopDetector(const LoopDetectorConfig& config) : config_(config) {}

void LoopDetector::Insert(std::shared_ptr<KeyFrame> kf) {
    if (!kf || !vocabulary_.IsLoaded())
        return;

    const auto& descriptors = kf->Descriptors();
    if (descriptors.empty())
        return;

    std::vector<float> bow;
    std::vector<std::pair<int, float>> weights;
    vocabulary_.Transform(descriptors, bow, weights);

    bow_vectors_[kf->Id()] = std::move(bow);
    word_weights_[kf->Id()] = std::move(weights);
    database_size_++;
}

bool LoopDetector::LoadVocabulary(const std::string& path) {
    return vocabulary_.Load(path);
}

std::shared_ptr<KeyFrame> LoopDetector::Detect(std::shared_ptr<KeyFrame> current_kf,
                                               const Map& map) {
    if (!current_kf)
        return nullptr;

    // If vocabulary is loaded, use BoW; otherwise use fallback
    if (!vocabulary_.IsLoaded()) {
        return DetectWithFallback(current_kf, map);
    }

    // Compute BoW vector for current KF
    const auto& descriptors = current_kf->Descriptors();
    std::vector<float> cur_bow;
    std::vector<std::pair<int, float>> cur_weights;
    vocabulary_.Transform(descriptors, cur_bow, cur_weights);

    // Query database for best match
    auto all_kfs = map.GetAllKeyFrames();
    double best_score = config_.min_score;
    std::shared_ptr<KeyFrame> best_kf = nullptr;

    // Exclude recent and covisible keyframes
    std::set<FrameId> recent_ids;
    auto recent_kfs = map.GetRecentKeyFrames(config_.max_recent_kfs);
    for (const auto& kf : recent_kfs) {
        if (kf)
            recent_ids.insert(kf->Id());
    }

    auto covisibles = current_kf->GetCovisiblesByWeight(1);
    for (auto* cov_kf : covisibles) {
        if (cov_kf)
            recent_ids.insert(cov_kf->Id());
    }

    for (const auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;
        if (kf->Id() == current_kf->Id())
            continue;
        if (recent_ids.contains(kf->Id()))
            continue;

        auto it = bow_vectors_.find(kf->Id());
        if (it == bow_vectors_.end())
            continue;

        double score = vocabulary_.Score(cur_bow, it->second);
        if (score > best_score) {
            best_score = score;
            best_kf = kf;
        }
    }

    // Consecutive detection check
    if (best_kf) {
        if (best_kf->Id() == last_candidate_id_) {
            consecutive_count_++;
        } else {
            consecutive_count_ = 1;
            last_candidate_id_ = best_kf->Id();
        }

        if (consecutive_count_ < config_.min_consecutive) {
            return nullptr;
        }
        return best_kf;
    }

    consecutive_count_ = 0;
    return nullptr;
}

std::shared_ptr<KeyFrame> LoopDetector::DetectWithFallback(std::shared_ptr<KeyFrame> current_kf,
                                                           const Map& map) {
    auto all_kfs = map.GetAllKeyFrames();
    double best_score = config_.min_inlier_ratio;
    std::shared_ptr<KeyFrame> best_kf = nullptr;

    std::set<FrameId> recent_ids;
    auto recent_kfs = map.GetRecentKeyFrames(config_.max_recent_kfs);
    for (const auto& kf : recent_kfs) {
        if (kf)
            recent_ids.insert(kf->Id());
    }

    auto covisibles = current_kf->GetCovisiblesByWeight(1);
    for (auto* cov_kf : covisibles) {
        if (cov_kf)
            recent_ids.insert(cov_kf->Id());
    }

    for (const auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;
        if (kf->Id() == current_kf->Id())
            continue;
        if (recent_ids.contains(kf->Id()))
            continue;

        double score =
            ComputeDescriptorSimilarity(current_kf->Descriptors(), kf->Descriptors(), 0.7f);

        if (score > best_score) {
            best_score = score;
            best_kf = kf;
        }
    }

    if (best_kf) {
        if (best_kf->Id() == last_candidate_id_) {
            consecutive_count_++;
        } else {
            consecutive_count_ = 1;
            last_candidate_id_ = best_kf->Id();
        }
        if (consecutive_count_ < config_.min_consecutive)
            return nullptr;
        return best_kf;
    }

    consecutive_count_ = 0;
    return nullptr;
}

double LoopDetector::ScoreBetween(std::shared_ptr<KeyFrame> kf1, std::shared_ptr<KeyFrame> kf2) {
    if (!kf1 || !kf2)
        return 0.0;

    auto it1 = bow_vectors_.find(kf1->Id());
    auto it2 = bow_vectors_.find(kf2->Id());

    if (it1 != bow_vectors_.end() && it2 != bow_vectors_.end()) {
        return vocabulary_.Score(it1->second, it2->second);
    }

    return ComputeDescriptorSimilarity(kf1->Descriptors(), kf2->Descriptors());
}

}  // namespace slamforge::loop_closing
