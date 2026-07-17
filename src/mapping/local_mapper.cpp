// =============================================================================
// LocalMapper implementation
// =============================================================================

#include "slamforge/mapping/local_mapper.h"

#include <algorithm>
#include <iostream>
#include <unordered_set>

#include "slamforge/core/camera.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"
#include "slamforge/features/orb_extractor.h"
#include "slamforge/geometry/triangulation.h"
#include "slamforge/optimization/ba.h"
#include "slamforge/tracking/matcher.h"

namespace slamforge::mapping {

LocalMapper::LocalMapper(Map& map, const Camera& camera, const MappingConfig& config,
                         features::OrbExtractor& extractor)
    : map_(map), camera_(camera), config_(config), extractor_(extractor) {}

LocalMapper::~LocalMapper() {
    Stop();
}

// ── Thread control ──────────────────────────────────────────────────────────

void LocalMapper::Start() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (running_)
        return;

    // A worker that has completed remains joinable until it is joined.  Join
    // it before assigning a replacement thread, otherwise a Start/Stop/Start
    // sequence terminates the process.
    if (thread_.joinable()) {
        thread_.join();
    }

    {
        std::lock_guard<std::mutex> queue_lock(mutex_);
        stop_requested_ = false;
    }
    is_finished_ = false;
    running_ = true;
    thread_ = std::thread(&LocalMapper::Run, this);
}

void LocalMapper::Stop() {
    RequestStop();
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (thread_.joinable()) {
        thread_.join();
    }
}

void LocalMapper::RequestStop() {
    {
        // Change the wait predicate while holding the same mutex used by
        // cv_.wait(). This prevents a stop notification from being lost
        // between the worker's predicate check and entering the wait state.
        std::lock_guard<std::mutex> lock(mutex_);
        stop_requested_ = true;
    }
    cv_.notify_all();
}

void LocalMapper::InsertKeyFrame(std::shared_ptr<KeyFrame> kf) {
    if (!accept_keyframes_)
        return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        new_keyframes_.push(std::move(kf));
        ++pending_work_;
    }
    cv_.notify_one();
}

int LocalMapper::QueueSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(new_keyframes_.size());
}

void LocalMapper::WaitUntilIdle() {
    std::unique_lock<std::mutex> lock(mutex_);
    idle_cv_.wait(lock, [this] { return pending_work_ == 0; });
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
        auto graph_lock = map_.AcquireGraphLock();
        try {
            ProcessNewKeyFrame();
            CreateNewMapPoints();
            CullMapPoints();
            LocalBundleAdjustment();
            // Keyframe culling remains disabled until it can atomically erase
            // landmark observations and graph edges.  Merely setting IsBad,
            // as the old implementation did, left stale observations behind
            // and destroyed the temporal chain needed for recovery.

            // Periodically clean up bad map points from the map
            {
                auto all_mps = map_.GetAllMapPoints();
                for (const auto& mp : all_mps) {
                    if (mp && mp->IsEraseReady(config_.min_observations)) {
                        map_.EraseMapPoint(mp->Id());
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[LocalMapper] Exception: " << e.what() << "\n";
        }

        current_kf_.reset();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (pending_work_ > 0) {
                --pending_work_;
            }
            if (pending_work_ == 0) {
                idle_cv_.notify_all();
            }
        }
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
    // Triangulating against every BA-window keyframe repeats expensive full
    // descriptor matching while adding little baseline diversity.  The five
    // strongest covisible neighbours are sufficient and match ORB-SLAM's
    // local-neighbour strategy for monocular mapping.
    auto covisibles = current_kf_->GetBestCovisibilityKeyFrames(
        std::min(5, static_cast<int>(config_.sliding_window_size)));

    // Always include a few temporal neighbours.  A frame tracked by the
    // essential-matrix fallback may not share an existing landmark yet, so it
    // has no covisibility edge until these pairs are triangulated.
    std::unordered_set<KeyFrame*> selected(covisibles.begin(), covisibles.end());
    auto recent = map_.GetRecentKeyFrames(8);
    for (auto it = recent.rbegin(); it != recent.rend() && covisibles.size() < 8; ++it) {
        KeyFrame* candidate = it->get();
        if (!candidate || candidate == current_kf_.get() || candidate->IsBad() ||
            !selected.insert(candidate).second) {
            continue;
        }
        covisibles.push_back(candidate);
    }

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
        if (!mp)
            continue;

        // Advance the recent-point grace period once per keyframe processed.
        mp->IncrementFrame();

        // IsEraseReady keeps valid two-view initialization points alive until
        // there is enough age and tracking evidence to judge their quality.
        if (mp->IsEraseReady(config_.min_observations)) {
            map_.EraseMapPoint(mp->Id());
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
    // Ceres is mathematically insensitive to residual insertion order, but
    // finite-precision Schur elimination is not. Newest-first IDs retain the
    // best-conditioned active landmarks first and, unlike unordered pointer
    // iteration, produce identical long-video results across runs.
    std::sort(map_points.begin(), map_points.end(),
              [](const MapPoint* lhs, const MapPoint* rhs) { return lhs->Id().id > rhs->Id().id; });

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
    if (!current_kf_)
        return;

    // Redundancy is a local property.  Re-scanning every keyframe and every
    // feature after each insertion was quadratic in map size.
    auto local_candidates =
        current_kf_->GetBestCovisibilityKeyFrames(static_cast<int>(config_.sliding_window_size));
    for (auto* kf : local_candidates) {
        if (!kf || kf->IsBad())
            continue;

        // Don't cull the current or most recent KF
        if (kf == current_kf_.get())
            continue;

        int total_mps = 0;
        int redundant_mps = 0;

        for (int i = 0; i < kf->NumKeyPoints(); ++i) {
            MapPointId mp_id = kf->MapPointIdAt(i);
            if (mp_id.id == 0)
                continue;

            total_mps++;
            const auto mp = map_.GetMapPoint(mp_id);
            if (mp && !mp->IsBad() && mp->Observations() >= 4) {
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

    // Mirror the already-computed weights.  The old implementation repeated
    // the complete quadratic feature comparison a second time.
    for (auto* other : kf->GetCovisiblesByWeight(1)) {
        if (other && !other->IsBad()) {
            other->AddConnection(kf.get(), kf->GetWeight(other));
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

}  // namespace slamforge::mapping
