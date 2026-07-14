// =============================================================================
// LoopClosing implementation — orchestrator thread
// =============================================================================

#include "litevo/loop_closing/loop_closing.h"

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
      detector_(LoopDetectorConfig{}),
      verifier_(LoopVerifierConfig{}, &camera),
      pose_graph_(PoseGraphConfig{}) {}

LoopClosing::~LoopClosing() {
    Stop();
}

// ── Thread control ──────────────────────────────────────────────────────────

void LoopClosing::Start() {
    if (running_)
        return;
    stop_requested_ = false;
    is_finished_ = false;
    running_ = true;
    thread_ = std::thread(&LoopClosing::Run, this);
}

void LoopClosing::Stop() {
    RequestStop();
    if (thread_.joinable()) {
        thread_.join();
    }
    // Also stop global BA if running
    if (global_ba_.IsRunning()) {
        global_ba_.RequestStop();
    }
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

        try {
            // ── Step 1: Add to BoW database ────────────────────────────────
            detector_.Insert(kf);

            // ── Step 2: Detect loop candidates ────────────────────────────
            auto candidate = detector_.Detect(kf, map_);

            if (!candidate) {
                continue;  // No candidate found
            }

            // ── Step 3: Verify geometrically ────────────────────────────────
            auto verification = verifier_.Verify(kf, candidate, detector_, map_);

            if (!verification.valid) {
                continue;  // Verification failed
            }

            std::cout << "[LoopClosing] Loop detected! "
                      << "Current KF #" << kf->Id().id << " matches KF #" << candidate->Id().id
                      << " with " << verification.num_inliers << " inliers"
                      << " (scale=" << verification.S_cw.s << ")\n";

            // ── Step 4: Correct the loop ──────────────────────────────────
            corrector_.CorrectLoop(kf, candidate, verification.S_cw, verification.matches,
                                   verification.matched_mps_cur, verification.matched_mps_cand,
                                   map_);

            // ── Step 5: Pose graph optimization ───────────────────────────

            // Compute relative pose between matched and current KF
            SE3 relative_pose = geometry::RelativePose(candidate->Pose(), kf->Pose());

            // Information matrix (identity scaled by connection weight)
            Mat6 info = Mat6::Identity() * 100.0;

            pose_graph_.Optimize(map_, kf, candidate, relative_pose, info);

            // ── Step 6: Launch global BA in separate thread ────────────────

            if (config_.enable_global_ba && camera_.fx() > 0) {
                // Stop any existing global BA
                if (global_ba_.IsRunning()) {
                    global_ba_.RequestStop();
                }
                GlobalBAConfig gba_cfg;
                gba_cfg.max_iterations = config_.global_ba_iterations;
                global_ba_.Start(map_, camera_, gba_cfg);
            }

            // ── Statistics ──────────────────────────────────────────────────
            loop_detected_ = true;
            num_loops_closed_++;

        } catch (const std::exception& e) {
            std::cerr << "[LoopClosing] Exception: " << e.what() << '\n';
        }
    }

    // Wait for global BA to finish before exiting
    if (global_ba_.IsRunning()) {
        global_ba_.RequestStop();
    }

    is_finished_ = true;
    running_ = false;
}

}  // namespace litevo::loop_closing
