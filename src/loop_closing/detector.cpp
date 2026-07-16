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

namespace {

cv::Mat SubsampleDescriptors(const cv::Mat& descriptors) {
    if (descriptors.empty() || descriptors.rows <= 600) {
        return descriptors.clone();
    }

    const int stride = std::max(1, descriptors.rows / 600);
    cv::Mat sampled;
    for (int row = 0; row < descriptors.rows; row += stride) {
        sampled.push_back(descriptors.row(row));
    }
    return sampled;
}

bool SameCandidateRegion(FrameId lhs, FrameId rhs) {
    const uint64_t delta = lhs.id > rhs.id ? lhs.id - rhs.id : rhs.id - lhs.id;
    return delta <= 300;
}

}  // namespace

LoopDetector::LoopDetector(const LoopDetectorConfig& config) : config_(config) {}

void LoopDetector::Insert(std::shared_ptr<KeyFrame> kf) {
    if (!kf)
        return;

    const auto& descriptors = kf->Descriptors();
    if (descriptors.empty())
        return;

    fallback_descriptors_[kf->Id()] = SubsampleDescriptors(descriptors);
    ++database_size_;

    if (vocabulary_.IsLoaded()) {
        std::vector<float> bow;
        std::vector<std::pair<int, float>> weights;
        vocabulary_.Transform(descriptors, bow, weights);
        bow_vectors_[kf->Id()] = std::move(bow);
        word_weights_[kf->Id()] = std::move(weights);
    }
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
        if (current_kf->Id().id <= kf->Id().id ||
            current_kf->Id().id - kf->Id().id <
                static_cast<uint64_t>(config_.min_frame_separation))
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
    // Full descriptor fallback is intentionally evaluated every other
    // keyframe.  It remains independent of an external vocabulary without
    // monopolizing the loop-closing worker on long videos.
    if (database_size_ < config_.max_recent_kfs + 2 || database_size_ % 2 != 0) {
        return nullptr;
    }

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
        if (current_kf->Id().id <= kf->Id().id ||
            current_kf->Id().id - kf->Id().id <
                static_cast<uint64_t>(config_.min_frame_separation))
            continue;
        if (recent_ids.contains(kf->Id()))
            continue;

        const auto cur_desc_it = fallback_descriptors_.find(current_kf->Id());
        const auto candidate_desc_it = fallback_descriptors_.find(kf->Id());
        if (cur_desc_it == fallback_descriptors_.end() ||
            candidate_desc_it == fallback_descriptors_.end()) {
            continue;
        }

        const double score =
            ComputeDescriptorSimilarity(cur_desc_it->second, candidate_desc_it->second, 0.72f);

        if (score > best_score) {
            best_score = score;
            best_kf = kf;
        }
    }

    if (best_kf) {
        if (consecutive_count_ > 0 && SameCandidateRegion(best_kf->Id(), last_candidate_id_)) {
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

    const auto fallback_1 = fallback_descriptors_.find(kf1->Id());
    const auto fallback_2 = fallback_descriptors_.find(kf2->Id());
    if (fallback_1 != fallback_descriptors_.end() && fallback_2 != fallback_descriptors_.end()) {
        return ComputeDescriptorSimilarity(fallback_1->second, fallback_2->second, 0.72f);
    }
    return ComputeDescriptorSimilarity(kf1->Descriptors(), kf2->Descriptors(), 0.72f);
}

}  // namespace slamforge::loop_closing
