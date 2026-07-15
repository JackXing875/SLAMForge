// =============================================================================
// LoopClosing implementation — orchestrator thread
// =============================================================================

#include "litevo/loop_closing/loop_closing.h"

#include <exception>
#include <iostream>

#include "litevo/core/camera.h"
#include "litevo/core/keyframe.h"
#include "litevo/core/map.h"
#include "litevo/features/orb_extractor.h"
#include "litevo/geometry/se3.h"

namespace litevo::loop_closing {

LoopClosing::LoopClosing(Map& map, const Camera& camera, const LoopClosingConfig& config)
    : map_(map),
      camera_(camera),
      config_(config),
      detector_(LoopDetectorConfig{.min_score = config.min_similarity_score,
                                   .min_consecutive = config.min_consecutive_loops}),
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

        bool launch_global_ba = false;
        GlobalBAConfig global_ba_config;
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

                std::cout << "[LoopClosing] Loop detected! "
                          << "Current KF #" << kf->Id().id << " matches KF #"
                          << candidate->Id().id << " with " << verification.num_inliers
                          << " inliers"
                          << " (scale=" << verification.S_cw.s << ")\n";

                // Preserve the pre-correction loop constraint for the pose
                // graph.  Reading it after CorrectLoop would encode an
                // already-mutated pose pair rather than the observation.
                const SE3 relative_pose = geometry::RelativePose(candidate->Pose(), kf->Pose());

                // ── Step 4: Correct the loop ──────────────────────────────
                corrector_.CorrectLoop(kf, candidate, verification.S_cw, verification.matches,
                                       verification.matched_mps_cur, verification.matched_mps_cand,
                                       map_);

                // ── Step 5: Pose graph optimization ───────────────────────
                const Mat6 info = Mat6::Identity() * 100.0;
                pose_graph_.Optimize(map_, kf, candidate, relative_pose, info);

                // GlobalBA::Start may join a prior worker.  Defer it until
                // this graph lock is released so a prior BA waiting for that
                // lock cannot deadlock the loop-closing worker.
                launch_global_ba = config_.enable_global_ba && camera_.fx() > 0;
                global_ba_config.max_iterations = config_.global_ba_iterations;

                // ── Statistics ──────────────────────────────────────────────
                loop_detected_ = true;
                num_loops_closed_++;
            }

            if (launch_global_ba) {
                global_ba_.Start(map_, camera_, global_ba_config);
            }
        } catch (const std::exception& e) {
            std::cerr << "[LoopClosing] Exception: " << e.what() << '\n';
        }
    }

    // Do not leave a BA worker mutating the map after loop closing stops.
    global_ba_.Stop();

}

}  // namespace litevo::loop_closing
