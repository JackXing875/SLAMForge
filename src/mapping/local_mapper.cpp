// =============================================================================
// LocalMapper implementation
// =============================================================================

#include "litevo/mapping/local_mapper.h"

#include <algorithm>
#include <iostream>
#include <unordered_set>

#include "litevo/core/camera.h"
#include "litevo/core/keyframe.h"
#include "litevo/core/map.h"
#include "litevo/core/map_point.h"
#include "litevo/features/orb_extractor.h"
#include "litevo/geometry/triangulation.h"
#include "litevo/optimization/ba.h"
#include "litevo/tracking/matcher.h"

namespace litevo::mapping {

LocalMapper::LocalMapper(Map& map, const Camera& camera, const MappingConfig& config,
                         features::OrbExtractor& extractor)
    : map_(map), camera_(camera), config_(config), extractor_(extractor) {}

LocalMapper::~LocalMapper() {
    Stop();
}

// ── Thread control ──────────────────────────────────────────────────────────

void LocalMapper::Start() {
    if (running_)
        return;
    stop_requested_ = false;
    is_finished_ = false;
    running_ = true;
    thread_ = std::thread(&LocalMapper::Run, this);
}

void LocalMapper::Stop() {
    RequestStop();
    if (thread_.joinable()) {
        thread_.join();
    }
}

void LocalMapper::RequestStop() {
    stop_requested_ = true;
    cv_.notify_all();
}

void LocalMapper::InsertKeyFrame(std::shared_ptr<KeyFrame> kf) {
    if (!accept_keyframes_)
        return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        new_keyframes_.push(std::move(kf));
    }
    cv_.notify_one();
}

int LocalMapper::QueueSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(new_keyframes_.size());
}

// ── Main loop ───────────────────────────────────────────────────────────────

void LocalMapper::Run() {
    while (true) {
        // Wait for a new keyframe
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return !new_keyframes_.empty() || stop_requested_; });

            if (stop_requested_ && new_keyframes_.empty()) {
                break;
            }

            current_kf_ = new_keyframes_.front();
            new_keyframes_.pop();
        }

        // Process the new keyframe
        try {
            ProcessNewKeyFrame();
            CreateNewMapPoints();
            CullMapPoints();
            LocalBundleAdjustment();
            CullKeyFrames();

            // Periodically clean up bad map points from the map
            {
                auto all_mps = map_.GetAllMapPoints();
                for (const auto& mp : all_mps) {
                    if (mp && mp->IsBad(config_.min_observations)) {
                        map_.EraseMapPoint(mp->Id());
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[LocalMapper] Exception: " << e.what() << "\n";
        }

        current_kf_.reset();
    }

    is_finished_ = true;
    running_ = false;
}

// ── Processing steps ────────────────────────────────────────────────────────

void LocalMapper::ProcessNewKeyFrame() {
    if (!current_kf_)
        return;

    // Update covisibility graph
    UpdateCovisibilityGraph(current_kf_);

    // Set spanning tree parent: most covisible older keyframe
    auto covisibles = current_kf_->GetBestCovisibilityKeyFrames(20);
    for (auto* kf : covisibles) {
        if (kf->Id().id < current_kf_->Id().id && !kf->IsBad()) {
            current_kf_->SetParent(kf);
            kf->AddChild(current_kf_.get());
            break;
        }
    }
}

void LocalMapper::CreateNewMapPoints() {
    if (!current_kf_)
        return;

    // Get covisible keyframes
    auto covisibles =
        current_kf_->GetBestCovisibilityKeyFrames(static_cast<int>(config_.sliding_window_size));

    for (auto* kf : covisibles) {
        if (kf->IsBad())
            continue;

        std::vector<std::shared_ptr<MapPoint>> new_mps;
        int n_created = TriangulateWithKf(current_kf_.get(), kf, new_mps);

        // Update covisibility weights for the new points
        if (n_created > 0) {
            current_kf_->AddConnection(kf, current_kf_->GetWeight(kf) + n_created);
            kf->AddConnection(current_kf_.get(), kf->GetWeight(current_kf_.get()) + n_created);
        }
    }
}

void LocalMapper::CullMapPoints() {
    auto all_mps = map_.GetAllMapPoints();
    for (auto& mp : all_mps) {
        if (!mp || mp->IsBad())
            continue;

        // Cull map points that aren't found often enough
        float ratio = mp->GetFoundRatio();
        if (ratio >= 0.0f && ratio < 0.25f && mp->FramesSinceCreation() > 2) {
            // Mark as bad — will be erased by the periodic cleanup
        }

        // Check if erase-ready
        if (mp->IsEraseReady(config_.min_observations)) {
            // Will be erased in the periodic cleanup
        }
    }
}

void LocalMapper::LocalBundleAdjustment() {
    if (!current_kf_)
        return;

    // Collect local KFs: current + N best covisible
    auto covisibles =
        current_kf_->GetBestCovisibilityKeyFrames(static_cast<int>(config_.sliding_window_size));

    std::vector<KeyFrame*> local_kfs;
    local_kfs.push_back(current_kf_.get());
    for (auto* kf : covisibles) {
        if (!kf->IsBad()) {
            local_kfs.push_back(kf);
        }
    }

    if (local_kfs.size() < 2)
        return;

    // Collect map points observed by local KFs
    std::unordered_set<MapPoint*> mp_set;
    for (auto* kf : local_kfs) {
        for (int i = 0; i < kf->NumKeyPoints(); ++i) {
            MapPointId mp_id = kf->MapPointIdAt(i);
            if (mp_id.id == 0)
                continue;
            auto mp = map_.GetMapPoint(mp_id);
            if (mp && !mp->IsBad()) {
                mp_set.insert(mp.get());
            }
        }
    }

    if (mp_set.empty())
        return;

    std::vector<MapPoint*> map_points(mp_set.begin(), mp_set.end());

    // Collect fixed KFs: KFs that observe these MPs but aren't in local set
    std::unordered_set<KeyFrame*> local_set(local_kfs.begin(), local_kfs.end());
    std::vector<KeyFrame*> fixed_kfs;

    auto all_kfs = map_.GetAllKeyFrames();
    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;
        if (local_set.contains(kf.get()))
            continue;

        // Check if this KF observes any of our map points
        bool observes_any = false;
        for (int i = 0; i < kf->NumKeyPoints() && !observes_any; ++i) {
            MapPointId mp_id = kf->MapPointIdAt(i);
            if (mp_id.id == 0)
                continue;
            auto mp = map_.GetMapPoint(mp_id);
            if (mp && mp_set.contains(mp.get())) {
                observes_any = true;
            }
        }

        if (observes_any) {
            fixed_kfs.push_back(kf.get());
        }
    }

    // Run BA
    optimization::BAOptions ba_opts;
    ba_opts.max_iterations = 10;
    ba_opts.huber_delta = config_.max_reprojection_error;
    ba_opts.use_huber = true;

    optimization::LocalBundleAdjuster adjuster(camera_, ba_opts);
    adjuster.Optimize(local_kfs, fixed_kfs, map_points);
}

void LocalMapper::CullKeyFrames() {
    // Cull redundant keyframes: >90% of map points are seen by at least
    // 3 other keyframes in the same or finer scale
    auto all_kfs = map_.GetAllKeyFrames();
    if (all_kfs.size() < 3)
        return;

    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;

        // Don't cull the current or most recent KF
        if (kf.get() == current_kf_.get())
            continue;

        auto covisibles = kf->GetCovisiblesByWeight(1);
        if (covisibles.size() < 3)
            continue;

        int total_mps = 0;
        int redundant_mps = 0;

        for (int i = 0; i < kf->NumKeyPoints(); ++i) {
            MapPointId mp_id = kf->MapPointIdAt(i);
            if (mp_id.id == 0)
                continue;

            total_mps++;
            int kf_count = 0;
            for (auto* cov_kf : covisibles) {
                for (int j = 0; j < cov_kf->NumKeyPoints(); ++j) {
                    if (cov_kf->MapPointIdAt(j) == mp_id) {
                        kf_count++;
                        break;
                    }
                }
                if (kf_count >= 3)
                    break;
            }

            if (kf_count >= 3) {
                redundant_mps++;
            }
        }

        if (total_mps > 0) {
            double ratio = static_cast<double>(redundant_mps) / total_mps;
            if (ratio > 0.9) {
                kf->SetBad(true);
            }
        }
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────────

void LocalMapper::UpdateCovisibilityGraph(std::shared_ptr<KeyFrame> kf) {
    auto all_kfs = map_.GetAllKeyFrames();
    kf->UpdateConnections(all_kfs);

    // Update connections for KFs that share points with this one
    for (auto& other : all_kfs) {
        if (!other || other.get() == kf.get() || other->IsBad())
            continue;

        int shared = 0;
        for (int i = 0; i < kf->NumKeyPoints(); ++i) {
            MapPointId mp_id = kf->MapPointIdAt(i);
            if (mp_id.id == 0)
                continue;
            for (int j = 0; j < other->NumKeyPoints(); ++j) {
                if (other->MapPointIdAt(j) == mp_id) {
                    shared++;
                    break;
                }
            }
        }

        if (shared > 0) {
            kf->AddConnection(other.get(), shared);
            other->AddConnection(kf.get(), shared);
        }
    }
}

int LocalMapper::TriangulateWithKf(KeyFrame* kf1, KeyFrame* kf2,
                                   std::vector<std::shared_ptr<MapPoint>>& new_mps) {
    // Find matching features between the two keyframes
    std::vector<std::pair<int, int>> matches;
    int n_matches = SearchForTriangulation(kf1, kf2, matches);

    if (n_matches < 20)
        return 0;

    // Build projection matrices
    Mat34 P1 = camera_.ProjectionMatrix(kf1->Pose());
    Mat34 P2 = camera_.ProjectionMatrix(kf2->Pose());

    Vec3 C1 = kf1->CameraCenter();
    Vec3 C2 = kf2->CameraCenter();

    // Check baseline / parallax
    double baseline = (C2 - C1).norm();
    if (baseline < 1e-6)
        return 0;

    geometry::TriangulationOptions tri_opts;
    tri_opts.min_parallax_deg = 0.5;  // degrees
    tri_opts.max_reprojection_px = config_.max_reprojection_error;
    tri_opts.min_depth = 0.01;
    tri_opts.max_depth = 200.0;

    const auto& kps1_undist = kf1->KeyPointsUndistorted();
    const auto& kps2_undist = kf2->KeyPointsUndistorted();

    int created = 0;

    for (const auto& [idx1, idx2] : matches) {
        // Skip if either already has a map point
        if (kf1->MapPointIdAt(idx1).id != 0 || kf2->MapPointIdAt(idx2).id != 0) {
            continue;
        }

        // Use undistorted keypoints for triangulation (as rays from camera)
        Vec2 pt1(kps1_undist[static_cast<size_t>(idx1)].pt.x,
                 kps1_undist[static_cast<size_t>(idx1)].pt.y);
        Vec2 pt2(kps2_undist[static_cast<size_t>(idx2)].pt.x,
                 kps2_undist[static_cast<size_t>(idx2)].pt.y);

        auto result = geometry::TriangulatePoint(pt1, pt2, P1, P2, C1, C2, tri_opts);

        if (!result.valid)
            continue;

        // Check depth in both cameras
        Vec3 p_cam1 = kf1->Pose() * result.point_w;
        Vec3 p_cam2 = kf2->Pose() * result.point_w;
        if (p_cam1.z() <= 0 || p_cam2.z() <= 0)
            continue;

        // Create the map point through the map
        auto mp = map_.AddMapPoint(result.point_w, kf1->Id());

        // Set descriptor from the keyframe
        mp->SetDescriptor(kf1->Descriptors().row(idx1));

        // Associate with both keyframes
        kf1->SetMapPointId(idx1, mp->Id());
        kf2->SetMapPointId(idx2, mp->Id());

        // Add observations
        mp->AddObservation(kf1->Id());
        mp->AddObservation(kf2->Id());
        mp->UpdateNormal(C1);

        new_mps.push_back(mp);
        created++;
    }

    return created;
}

int LocalMapper::SearchForTriangulation(KeyFrame* kf1, KeyFrame* kf2,
                                        std::vector<std::pair<int, int>>& matches) const {
    // Match descriptors between the two KFs
    tracking::FeatureMatcher matcher;
    auto raw_matches =
        matcher.MatchByDescriptor(kf1->Descriptors(), kf2->Descriptors(), 0.6f, true);

    matches = std::move(raw_matches);
    return static_cast<int>(matches.size());
}

}  // namespace litevo::mapping
