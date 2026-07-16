// =============================================================================
// LoopClosing implementation — orchestrator thread
// =============================================================================

#include "slamforge/loop_closing/loop_closing.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "slamforge/core/camera.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/features/orb_extractor.h"
#include "slamforge/geometry/se3.h"

namespace slamforge::loop_closing {

LoopClosing::LoopClosing(Map& map, const Camera& camera, const LoopClosingConfig& config)
    : map_(map),
      camera_(camera),
      config_(config),
      detector_(LoopDetectorConfig{.min_score = config.min_similarity_score,
                                   .min_consecutive = config.min_consecutive_loops,
                                   .min_inlier_ratio = config.fallback_min_match_ratio,
                                   .min_frame_separation = config.min_frame_separation}),
      verifier_(LoopVerifierConfig{}, &camera),
      pose_graph_(PoseGraphConfig{.max_iterations = config.pose_graph_iterations}) {}

LoopClosing::~LoopClosing() {
    Stop();
}

// ── Thread control ──────────────────────────────────────────────────────────

void LoopClosing::Start() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (running_)
        return;

    // A finished std::thread remains joinable.  Join it before assigning a
    // replacement, otherwise assigning to thread_ would call std::terminate.
    if (thread_.joinable()) {
        thread_.join();
    }

    stop_requested_ = false;
    is_finished_ = false;
    loop_detected_ = false;
    num_loops_closed_ = 0;
    {
        std::lock_guard<std::mutex> correction_lock(correction_mutex_);
        pending_loops_.clear();
        applied_corrections_.clear();
        final_search_done_ = false;
    }
    running_ = true;
    try {
        thread_ = std::thread([this] {
            try {
                Run();
            } catch (const std::exception& e) {
                std::cerr << "[LoopClosing] Worker exception: " << e.what() << '\n';
            } catch (...) {
                std::cerr << "[LoopClosing] Worker exception: unknown error\n";
            }
            running_ = false;
            is_finished_ = true;
        });
    } catch (...) {
        running_ = false;
        is_finished_ = true;
        throw;
    }
}

void LoopClosing::Stop() {
    RequestStop();
    {
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
        if (thread_.joinable()) {
            thread_.join();
        }
    }
    global_ba_.Stop();
    SearchBestGlobalLoop();
    ApplyCorrections();
}

void LoopClosing::RequestStop() {
    stop_requested_ = true;
    cv_.notify_all();
}

void LoopClosing::InsertKeyFrame(std::shared_ptr<KeyFrame> kf) {
    if (!kf)
        return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        kf_queue_.push(std::move(kf));
    }
    cv_.notify_one();
}

int LoopClosing::QueueSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(kf_queue_.size());
}

// ── Vocabulary ──────────────────────────────────────────────────────────────

bool LoopClosing::LoadVocabulary(const std::string& path) {
    if (path.empty() || running_) {
        return false;
    }
    return detector_.LoadVocabulary(path);
}

// ── Main loop ───────────────────────────────────────────────────────────────

void LoopClosing::Run() {
    while (true) {
        // Wait for a new keyframe
        std::shared_ptr<KeyFrame> kf;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return !kf_queue_.empty() || stop_requested_; });

            if (stop_requested_ && kf_queue_.empty()) {
                break;
            }

            kf = kf_queue_.front();
            kf_queue_.pop();
        }

        if (!kf)
            continue;

        // LoopDetected() describes the last processed keyframe, not an
        // historical latch that stays true after a loop was closed.
        loop_detected_ = false;

        try {
            {
                // Frame/KeyFrame fields are not individually synchronized.
                // Serialize this full graph transaction against tracking,
                // local BA, pose-graph updates, and GlobalBA.
                auto graph_lock = map_.AcquireGraphLock();

                // ── Step 1: Add to BoW database ────────────────────────────
                detector_.Insert(kf);

                // ── Step 2: Detect loop candidates ────────────────────────
                auto candidate = detector_.Detect(kf, map_);

                if (!candidate) {
                    continue;  // No candidate found
                }

                // ── Step 3: Verify geometrically ────────────────────────────
                auto verification = verifier_.Verify(kf, candidate, detector_, map_);

                if (!verification.valid) {
                    continue;  // Verification failed
                }

                std::cout << "[LoopClosing] Verified loop candidate: "
                          << "Current KF #" << kf->Id().id << " matches KF #" << candidate->Id().id
                          << " with " << verification.num_inliers << " inliers"
                          << " (scale=" << verification.S_cw.s << ")\n";

                // Keep the strongest verified closure.  Applying multiple
                // near-duplicate closures from the same return-to-start event
                // would compound the same Sim(3) correction.  Finalization is
                // deferred until tracking has stopped so the live motion model
                // never changes coordinate systems mid-frame.
                AddPendingLoop(PendingLoop{kf, candidate, verification});
                loop_detected_ = true;
            }
        } catch (const std::exception& e) {
            std::cerr << "[LoopClosing] Exception: " << e.what() << '\n';
        }
    }

    // Do not leave a BA worker mutating the map after loop closing stops.
    global_ba_.Stop();
}

void LoopClosing::AddPendingLoop(PendingLoop pending) {
    if (!pending.current || !pending.candidate || !pending.verification.valid ||
        pending.verification.num_inliers < 6) {
        return;
    }

    constexpr uint64_t kSameLoopEventFrames = 500;
    std::lock_guard<std::mutex> lock(correction_mutex_);
    for (auto& existing : pending_loops_) {
        const uint64_t existing_frame = existing.current->Id().id;
        const uint64_t pending_frame = pending.current->Id().id;
        const uint64_t separation = existing_frame > pending_frame ? existing_frame - pending_frame
                                                                   : pending_frame - existing_frame;
        if (separation <= kSameLoopEventFrames) {
            if (pending.verification.num_inliers > existing.verification.num_inliers) {
                existing = std::move(pending);
            }
            return;
        }
    }
    pending_loops_.push_back(std::move(pending));
}

void LoopClosing::SearchBestGlobalLoop() {
    {
        std::lock_guard<std::mutex> lock(correction_mutex_);
        if (final_search_done_) {
            final_search_done_ = true;
            return;
        }
        final_search_done_ = true;
    }

    auto graph_lock = map_.AcquireGraphLock();
    const auto keyframes = map_.GetAllKeyFrames();
    if (keyframes.size() < 40) {
        return;
    }

    struct ScoredPair {
        double score = 0.0;
        std::shared_ptr<KeyFrame> current;
        std::shared_ptr<KeyFrame> candidate;
    };
    std::vector<ScoredPair> scored_pairs;

    // Batch-video finalization concentrates on the newest 30% of the route,
    // where return-to-start loops normally appear.  Older candidates are
    // sampled at stride two; the two initialization keyframes are always
    // retained because they are especially valuable anchors.
    const size_t current_begin = keyframes.size() * 8 / 10;
    const size_t candidate_end = std::max<size_t>(2, keyframes.size() * 35 / 100);
    for (size_t current_index = current_begin; current_index < keyframes.size(); ++current_index) {
        const auto& current = keyframes[current_index];
        if (!current || current->IsBad()) {
            continue;
        }
        for (size_t candidate_index = 0; candidate_index < candidate_end; candidate_index += 2) {
            const auto& candidate = keyframes[candidate_index];
            if (!candidate || candidate->IsBad()) {
                continue;
            }
            if (current->Id().id <= candidate->Id().id ||
                current->Id().id - candidate->Id().id <
                    static_cast<uint64_t>(config_.min_frame_separation)) {
                continue;
            }
            const double score = detector_.ScoreBetween(current, candidate);
            if (score >= config_.fallback_min_match_ratio) {
                scored_pairs.push_back({score, current, candidate});
            }
        }
        if (current_index > 21 && keyframes[1] && !keyframes[1]->IsBad() &&
            current->Id().id > keyframes[1]->Id().id &&
            current->Id().id - keyframes[1]->Id().id >=
                static_cast<uint64_t>(config_.min_frame_separation)) {
            const double score = detector_.ScoreBetween(current, keyframes[1]);
            if (score >= config_.fallback_min_match_ratio) {
                scored_pairs.push_back({score, current, keyframes[1]});
            }
        }
    }

    std::sort(scored_pairs.begin(), scored_pairs.end(),
              [](const ScoredPair& lhs, const ScoredPair& rhs) { return lhs.score > rhs.score; });
    // Avoid letting one highly textured revisit consume the entire verifier
    // budget. Retain the strongest candidates from each temporal region so a
    // later return-to-start closure is still tested.
    std::unordered_map<uint64_t, int> pairs_per_region;
    std::vector<ScoredPair> distributed_pairs;
    distributed_pairs.reserve(std::min<size_t>(scored_pairs.size(), 240));
    for (auto& pair : scored_pairs) {
        const uint64_t region = pair.current->Id().id / 300;
        if (pairs_per_region[region] >= 60) {
            continue;
        }
        ++pairs_per_region[region];
        distributed_pairs.push_back(std::move(pair));
        if (distributed_pairs.size() >= 240) {
            break;
        }
    }
    scored_pairs = std::move(distributed_pairs);

    std::optional<PendingLoop> best;
    std::vector<PendingLoop> verified_loops;
    double top_score = 0.0;
    VerificationResult top_diagnostic;
    std::shared_ptr<KeyFrame> top_current;
    std::shared_ptr<KeyFrame> top_candidate;
    for (const ScoredPair& pair : scored_pairs) {
        auto verification = verifier_.Verify(pair.current, pair.candidate, detector_, map_);
        if (pair.score > top_score) {
            top_score = pair.score;
            top_diagnostic = verification;
            top_current = pair.current;
            top_candidate = pair.candidate;
        }
        if (verification.valid) {
            PendingLoop verified{pair.current, pair.candidate, verification};
            verified_loops.push_back(verified);
            if (!best || verification.num_inliers > best->verification.num_inliers) {
                best = std::move(verified);
            }
        }
    }

    if (best) {
        std::cout << "[LoopClosing] Final global search verified KF #" << best->current->Id().id
                  << " -> KF #" << best->candidate->Id().id << " with "
                  << best->verification.num_inliers << " inliers"
                  << " (scale=" << best->verification.S_cw.s << ")\n";
        // A closure spanning the final return-to-start section is retained as
        // a separate global anchor, while an earlier independent revisit can
        // constrain the middle of the trajectory.
        for (auto& verified : verified_loops) {
            AddPendingLoop(std::move(verified));
        }
        loop_detected_ = true;
    } else if (!scored_pairs.empty()) {
        std::cout << "[LoopClosing] Final global search found no geometrically valid loop; "
                  << "top KF #" << (top_current ? top_current->Id().id : 0) << " -> KF #"
                  << (top_candidate ? top_candidate->Id().id : 0) << ", score=" << top_score
                  << ", matches=" << top_diagnostic.num_initial_matches
                  << ", geometric=" << top_diagnostic.num_geometric_matches
                  << ", 3D=" << top_diagnostic.num_3d_correspondences
                  << ", Sim3=" << top_diagnostic.num_sim3_inliers
                  << ", reprojection=" << top_diagnostic.num_reprojection_inliers
                  << ", scale=" << top_diagnostic.estimated_scale << '\n';
    }
}

std::optional<LoopCorrection> LoopClosing::BestCorrection() const {
    std::lock_guard<std::mutex> lock(correction_mutex_);
    if (!applied_corrections_.empty()) {
        return applied_corrections_.back();
    }
    if (pending_loops_.empty()) {
        return std::nullopt;
    }
    const auto best =
        std::max_element(pending_loops_.begin(), pending_loops_.end(),
                         [](const PendingLoop& lhs, const PendingLoop& rhs) {
                             return lhs.verification.num_inliers < rhs.verification.num_inliers;
                         });
    return LoopCorrection{best->candidate->Id(), best->current->Id(), best->verification.S_cw,
                          best->verification.num_inliers};
}

std::vector<LoopCorrection> LoopClosing::Corrections() const {
    std::lock_guard<std::mutex> lock(correction_mutex_);
    return applied_corrections_;
}

bool LoopClosing::ApplyCorrections() {
    std::vector<PendingLoop> pending;
    {
        std::lock_guard<std::mutex> lock(correction_mutex_);
        if (!applied_corrections_.empty()) {
            return true;
        }
        pending = pending_loops_;
    }
    if (pending.empty()) {
        return false;
    }

    std::sort(pending.begin(), pending.end(), [](const PendingLoop& lhs, const PendingLoop& rhs) {
        return lhs.current->Id().id < rhs.current->Id().id;
    });

    std::vector<LoopCorrection> applied;
    for (const PendingLoop& loop : pending) {
        // Every verifier result was measured in the original tracking map.
        // Earlier temporal corrections have already changed both endpoints,
        // so conjugate this raw constraint into the current map coordinates:
        // adjusted(C_cur p) = C_match(raw_loop(p)).
        const auto accumulated_at = [&applied](FrameId frame) {
            geometry::Sim3 accumulated = geometry::Sim3::Identity();
            for (const LoopCorrection& correction : applied) {
                accumulated = InterpolateLoopCorrection(correction, frame) * accumulated;
            }
            return accumulated;
        };
        const geometry::Sim3 candidate_coordinates = accumulated_at(loop.candidate->Id());
        const geometry::Sim3 current_coordinates = accumulated_at(loop.current->Id());
        const geometry::Sim3 adjusted =
            candidate_coordinates * loop.verification.S_cw * current_coordinates.Inverse();
        if (!adjusted.R.allFinite() || !adjusted.t.allFinite() || !std::isfinite(adjusted.s) ||
            adjusted.s < 0.01 || adjusted.s > 100.0) {
            continue;
        }

        corrector_.CorrectLoop(loop.current, loop.candidate, adjusted, loop.verification.matches,
                               loop.verification.matched_mps_cur,
                               loop.verification.matched_mps_cand, map_);

        const LoopCorrection correction{loop.candidate->Id(), loop.current->Id(), adjusted,
                                        loop.verification.num_inliers};
        applied.push_back(correction);
        std::cout << "[LoopClosing] Applied global correction across frames "
                  << correction.matched_frame.id << ".." << correction.current_frame.id
                  << " (inliers=" << correction.num_inliers << ", scale=" << correction.transform.s
                  << ")\n";
    }

    if (applied.empty()) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(correction_mutex_);
        applied_corrections_ = applied;
    }
    num_loops_closed_ = static_cast<int>(applied.size());
    return true;
}

bool LoopClosing::ApplyBestCorrection() {
    return ApplyCorrections();
}

}  // namespace slamforge::loop_closing
