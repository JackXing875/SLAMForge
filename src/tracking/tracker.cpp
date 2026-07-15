// =============================================================================
// Tracker implementation
// =============================================================================

#include "litevo/tracking/tracker.h"

#include <algorithm>

#include "litevo/core/camera.h"
#include "litevo/core/frame.h"
#include "litevo/core/keyframe.h"
#include "litevo/core/map_point.h"
#include "litevo/features/orb_extractor.h"
#include "litevo/geometry/pnp.h"
#include "litevo/geometry/se3.h"
#include "litevo/loop_closing/loop_closing.h"
#include "litevo/mapping/local_mapper.h"
#include "litevo/tracking/initializer.h"
#include "litevo/tracking/matcher.h"

namespace litevo::tracking {

Tracker::Tracker(const Camera& camera, const TrackingConfig& config, const OrbConfig& orb_config)
    : camera_(camera), config_(config), orb_config_(orb_config) {
    // Normal ORB extractor
    features::OrbExtractor::Options orb_opts;
    orb_opts.num_features = orb_config_.num_features;
    orb_opts.scale_factor = static_cast<double>(orb_config_.scale_factor);
    orb_opts.num_levels = orb_config_.num_levels;
    orb_opts.ini_threshold = orb_config_.ini_threshold;
    orb_opts.min_threshold = orb_config_.min_threshold;
    orb_opts.patch_size = orb_config_.patch_size;
    orb_extractor_ = std::make_unique<features::OrbExtractor>(orb_opts);

    // Initialization ORB extractor (double the features)
    orb_opts.num_features = orb_config_.num_features * 2;
    orb_extractor_ini_ = std::make_unique<features::OrbExtractor>(orb_opts);

    // Matcher
    matcher_ = std::make_unique<FeatureMatcher>();

    // Initializer
    MonocularInitializer::Options init_opts;
    init_opts.min_features = config_.min_features_for_tracking;
    init_opts.min_matches = config_.min_features_for_tracking;
    init_opts.min_parallax_deg = config_.min_parallax_deg;
    init_opts.max_reproj_error = config_.max_reprojection_error;
    initializer_ = std::make_unique<MonocularInitializer>(camera_, *orb_extractor_ini_, init_opts);
}

Tracker::~Tracker() = default;

cv::Mat Tracker::BuildK() const {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = camera_.fx();
    K.at<double>(1, 1) = camera_.fy();
    K.at<double>(0, 2) = camera_.cx();
    K.at<double>(1, 2) = camera_.cy();
    return K;
}

void Tracker::Reset() {
    auto graph_lock = map_.AcquireGraphLock();
    state_ = TrackingState::NOT_INITIALIZED;
    map_.Clear();
    current_frame_.reset();
    last_frame_.reset();
    reference_kf_.reset();
    velocity_ = SE3::Identity();
    has_velocity_ = false;
    num_tracked_points_ = 0;
    frames_since_last_kf_ = 0;
    is_new_keyframe_ = false;
    initializer_->Reset();
    Frame::ResetIdCounter();
    MapPoint::ResetIdCounter();
}

// ── Main tracking entry point ──────────────────────────────────────────────

std::optional<SE3> Tracker::Track(const cv::Mat& image, double timestamp) {
    // Build Frame (extract ORB features, undistort, build grid)
    features::OrbExtractor& extractor =
        (state_ == TrackingState::NOT_INITIALIZED || state_ == TrackingState::INITIALIZING)
            ? *orb_extractor_ini_
            : *orb_extractor_;

    current_frame_ = std::make_shared<Frame>(image, timestamp, extractor, camera_);
    is_new_keyframe_ = false;

    // Keyframe poses and feature-to-landmark associations are shared with the
    // mapping and loop-closing workers.  Keep one coherent graph snapshot for
    // this tracking update instead of racing a loop correction or BA write.
    auto graph_lock = map_.AcquireGraphLock();

    // ── NOT_INITIALIZED → attempt initialization ──────────────────────
    if (state_ == TrackingState::NOT_INITIALIZED) {
        state_ = TrackingState::INITIALIZING;

        auto result = initializer_->Initialize(*current_frame_);
        if (result.success) {
            CreateInitialMap(result);
            state_ = TrackingState::OK;
            last_frame_ = current_frame_;
            num_tracked_points_ = static_cast<int>(result.map_points.size());
            return current_frame_->Pose();
        }

        // Still waiting for second frame or initialization failed
        last_frame_ = current_frame_;
        return std::nullopt;
    }

    // ── INITIALIZING → continue initialization attempt ─────────────────
    if (state_ == TrackingState::INITIALIZING) {
        auto result = initializer_->Initialize(*current_frame_);
        if (result.success) {
            CreateInitialMap(result);
            state_ = TrackingState::OK;
            last_frame_ = current_frame_;
            num_tracked_points_ = static_cast<int>(result.map_points.size());
            return current_frame_->Pose();
        }

        last_frame_ = current_frame_;
        return std::nullopt;
    }

    // ── OK → normal tracking ───────────────────────────────────────────
    if (state_ == TrackingState::OK) {
        // Reset tracking flags
        for (const auto& mp : map_.GetAllMapPoints()) {
            mp->ResetFound();
        }

        bool ok = false;

        // Stage 1: Motion model
        if (has_velocity_) {
            ok = TrackWithMotionModel();
        }

        // Stage 2: Reference keyframe fallback
        if (!ok) {
            ok = TrackReferenceKeyFrame();
        }

        // Stage 3: Relocalization
        if (!ok) {
            ok = Relocalization();
        }

        if (ok) {
            // Stage 4: Local map refinement
            TrackLocalMap();

            // Update motion model
            if (last_frame_) {
                UpdateMotionModel();
            }

            // Keyframe decision
            if (NeedNewKeyFrame()) {
                auto kf = std::make_shared<KeyFrame>(*current_frame_);
                map_.AddKeyFrame(kf);
                RegisterKeyFrameObservations(kf);
                reference_kf_ = kf;

                // Notify local mapper (if running)
                if (local_mapper_) {
                    local_mapper_->InsertKeyFrame(kf);
                }
                if (loop_closing_) {
                    loop_closing_->InsertKeyFrame(kf);
                }

                frames_since_last_kf_ = 0;
                is_new_keyframe_ = true;
            } else {
                frames_since_last_kf_++;
            }

            last_frame_ = current_frame_;
            return current_frame_->Pose();
        }

        // All stages failed
        state_ = TrackingState::LOST;
        last_frame_ = current_frame_;
        return std::nullopt;
    }

    // ── LOST → relocalization ──────────────────────────────────────────
    if (state_ == TrackingState::LOST) {
        bool ok = Relocalization();
        if (ok) {
            state_ = TrackingState::OK;
            last_frame_ = current_frame_;
            return current_frame_->Pose();
        }
        return std::nullopt;
    }

    return std::nullopt;
}

// ── Track with motion model ────────────────────────────────────────────────

bool Tracker::TrackWithMotionModel() {
    if (!last_frame_ || !has_velocity_) {
        return false;
    }

    // Predict pose: Tcw = velocity * last_Tcw
    SE3 predicted_pose = velocity_ * last_frame_->Pose();
    current_frame_->SetPose(predicted_pose);

    // Collect map points from last frame
    std::vector<std::shared_ptr<MapPoint>> last_mps;

    for (int i = 0; i < last_frame_->NumKeyPoints(); ++i) {
        MapPointId mp_id = last_frame_->MapPointIdAt(i);
        if (mp_id.id != 0) {
            auto mp = map_.GetMapPoint(mp_id);
            if (mp && !mp->IsBad()) {
                last_mps.push_back(mp);
            }
        }
    }

    if (last_mps.empty()) {
        return false;
    }

    // Search by projection
    auto matches = matcher_->SearchByProjection(*current_frame_, last_mps, camera_,
                                                static_cast<float>(config_.max_reprojection_error));

    if (static_cast<int>(matches.size()) < config_.min_features_for_tracking / 2) {
        // Try again with wider search radius
        matches =
            matcher_->SearchByProjection(*current_frame_, last_mps, camera_,
                                         static_cast<float>(config_.max_reprojection_error) * 2.0f);
    }

    if (static_cast<int>(matches.size()) < config_.min_features_for_tracking / 3) {
        return false;
    }

    // Build 3D-2D correspondences
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    std::vector<std::shared_ptr<MapPoint>> matched_mps;
    const auto& kps = current_frame_->KeyPointsUndistorted();

    for (const auto& [kp_idx, mp_id] : matches) {
        auto mp = map_.GetMapPoint(mp_id);
        if (!mp || mp->IsBad())
            continue;

        const Vec3& pw = mp->Position();
        pts_3d.emplace_back(static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                            static_cast<float>(pw.z()));
        pts_2d.emplace_back(kps[static_cast<size_t>(kp_idx)].pt.x,
                            kps[static_cast<size_t>(kp_idx)].pt.y);
        matched_mps.push_back(mp);

        // Set association
        current_frame_->SetMapPointId(kp_idx, mp_id);
    }

    // PnP + refine
    SE3 pose = predicted_pose;
    if (!EstimatePose(pts_3d, pts_2d, pose, 10)) {
        return false;
    }

    current_frame_->SetPose(pose);
    for (const auto& mp : matched_mps) {
        mp->IncreaseFound();
        mp->SetFound(true);
    }
    num_tracked_points_ = static_cast<int>(matched_mps.size());
    return true;
}

// ── Track reference keyframe ───────────────────────────────────────────────

bool Tracker::TrackReferenceKeyFrame() {
    if (!reference_kf_) {
        return false;
    }

    // Match descriptors
    auto matches = matcher_->MatchByDescriptor(reference_kf_->Descriptors(),
                                               current_frame_->Descriptors(), 0.7f, true);

    if (static_cast<int>(matches.size()) < config_.min_features_for_tracking / 2) {
        return false;
    }

    // Build 3D-2D correspondences from matched map points
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    std::vector<std::shared_ptr<MapPoint>> matched_mps;
    const auto& kps = current_frame_->KeyPointsUndistorted();

    for (const auto& [ref_idx, cur_idx] : matches) {
        MapPointId mp_id = reference_kf_->MapPointIdAt(ref_idx);
        if (mp_id.id == 0)
            continue;

        auto mp = map_.GetMapPoint(mp_id);
        if (!mp || mp->IsBad())
            continue;

        const Vec3& pw = mp->Position();
        pts_3d.emplace_back(static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                            static_cast<float>(pw.z()));
        pts_2d.emplace_back(kps[static_cast<size_t>(cur_idx)].pt.x,
                            kps[static_cast<size_t>(cur_idx)].pt.y);
        matched_mps.push_back(mp);

        current_frame_->SetMapPointId(cur_idx, mp_id);
    }

    if (static_cast<int>(pts_3d.size()) < 10) {
        return false;
    }

    // Use last frame's pose as initial guess
    SE3 pose = last_frame_ ? last_frame_->Pose() : SE3::Identity();

    if (!EstimatePose(pts_3d, pts_2d, pose, 10)) {
        return false;
    }

    current_frame_->SetPose(pose);
    for (const auto& mp : matched_mps) {
        mp->IncreaseVisible();
        mp->IncreaseFound();
        mp->SetFound(true);
    }
    num_tracked_points_ = static_cast<int>(matched_mps.size());
    return true;
}

// ── Track local map ────────────────────────────────────────────────────────

bool Tracker::TrackLocalMap() {
    // Collect all map points that could be visible
    auto all_mps = map_.GetAllMapPoints();

    // Filter out already-matched and bad points
    std::vector<std::shared_ptr<MapPoint>> candidates;
    for (const auto& mp : all_mps) {
        if (!mp || mp->IsBad())
            continue;
        if (mp->IsFound())
            continue;  // Already matched in motion model / ref KF stage
        candidates.push_back(mp);
    }

    if (candidates.empty()) {
        return true;  // Nothing to add, but tracking is already OK
    }

    // Project and match
    auto matches = matcher_->SearchByProjection(*current_frame_, candidates, camera_,
                                                static_cast<float>(config_.max_reprojection_error));

    // If we got additional matches, rebuild correspondence set and refine
    if (!matches.empty()) {
        // Collect ALL 3D-2D correspondences (existing + new)
        std::vector<cv::Point3f> pts_3d;
        std::vector<cv::Point2f> pts_2d;
        const auto& kps = current_frame_->KeyPointsUndistorted();

        // Add existing matches
        for (int i = 0; i < current_frame_->NumKeyPoints(); ++i) {
            MapPointId mp_id = current_frame_->MapPointIdAt(i);
            if (mp_id.id == 0)
                continue;
            auto mp = map_.GetMapPoint(mp_id);
            if (!mp || mp->IsBad())
                continue;

            const Vec3& pw = mp->Position();
            pts_3d.emplace_back(static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                                static_cast<float>(pw.z()));
            pts_2d.emplace_back(kps[static_cast<size_t>(i)].pt.x, kps[static_cast<size_t>(i)].pt.y);
        }

        // Add new matches from local map
        std::vector<std::shared_ptr<MapPoint>> newly_matched_mps;
        for (const auto& [kp_idx, mp_id] : matches) {
            auto mp = map_.GetMapPoint(mp_id);
            if (!mp || mp->IsBad())
                continue;

            const Vec3& pw = mp->Position();
            pts_3d.emplace_back(static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                                static_cast<float>(pw.z()));
            pts_2d.emplace_back(kps[static_cast<size_t>(kp_idx)].pt.x,
                                kps[static_cast<size_t>(kp_idx)].pt.y);

            current_frame_->SetMapPointId(kp_idx, mp_id);
            newly_matched_mps.push_back(mp);
        }

        // Refine pose with all correspondences
        if (pts_3d.size() >= 10) {
            SE3 pose = current_frame_->Pose();
            if (EstimatePose(pts_3d, pts_2d, pose, 10)) {
                current_frame_->SetPose(pose);
            }

            num_tracked_points_ = std::max(num_tracked_points_, static_cast<int>(pts_3d.size()));
        }

        for (const auto& mp : newly_matched_mps) {
            mp->IncreaseFound();
            mp->SetFound(true);
        }
    }

    return true;
}

// ── Relocalization ─────────────────────────────────────────────────────────

bool Tracker::Relocalization() {
    // Try matching against recent keyframes
    auto recent_kfs = map_.GetRecentKeyFrames(5);

    if (recent_kfs.empty()) {
        return false;
    }

    for (const auto& kf : recent_kfs) {
        if (!kf)
            continue;

        auto matches = matcher_->MatchByDescriptor(kf->Descriptors(), current_frame_->Descriptors(),
                                                   0.75f, true);

        if (static_cast<int>(matches.size()) < 15) {
            continue;
        }

        // Build 3D-2D correspondences
        std::vector<cv::Point3f> pts_3d;
        std::vector<cv::Point2f> pts_2d;
        std::vector<std::pair<int, std::shared_ptr<MapPoint>>> matched_mps;
        const auto& kps = current_frame_->KeyPointsUndistorted();

        for (const auto& [kf_idx, cur_idx] : matches) {
            MapPointId mp_id = kf->MapPointIdAt(kf_idx);
            if (mp_id.id == 0)
                continue;

            auto mp = map_.GetMapPoint(mp_id);
            if (!mp || mp->IsBad())
                continue;

            const Vec3& pw = mp->Position();
            pts_3d.emplace_back(static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                                static_cast<float>(pw.z()));
            pts_2d.emplace_back(kps[static_cast<size_t>(cur_idx)].pt.x,
                                kps[static_cast<size_t>(cur_idx)].pt.y);
            matched_mps.emplace_back(cur_idx, mp);
        }

        if (static_cast<int>(pts_3d.size()) < 15) {
            continue;
        }

        SE3 pose = SE3::Identity();
        if (EstimatePose(pts_3d, pts_2d, pose, 15)) {
            current_frame_->SetPose(pose);
            for (const auto& [cur_idx, mp] : matched_mps) {
                current_frame_->SetMapPointId(cur_idx, mp->Id());
                mp->IncreaseVisible();
                mp->IncreaseFound();
                mp->SetFound(true);
            }
            num_tracked_points_ = static_cast<int>(matched_mps.size());
            return true;
        }
    }

    return false;
}

// ── Keyframe decision ──────────────────────────────────────────────────────

bool Tracker::NeedNewKeyFrame() {
    if (num_tracked_points_ < 15) {
        return false;
    }

    int ref_map_points = reference_kf_ ? reference_kf_->NumMapPoints() : num_tracked_points_;
    if (ref_map_points == 0)
        ref_map_points = 1;

    // Condition 1: tracking weakened significantly (monocular thRefRatio = 0.9)
    bool c1 = num_tracked_points_ < static_cast<int>(ref_map_points * 0.9);

    // Condition 2: max frames exceeded
    bool c2 = frames_since_last_kf_ >= config_.max_frames_between_kf;

    // Condition 3: enough frames + weakening
    bool c3 = (frames_since_last_kf_ >= config_.min_frames_between_kf) && c1;

    return (c1 || c2 || c3);
}

// ── Create initial map ─────────────────────────────────────────────────────

void Tracker::CreateInitialMap(const InitializationResult& result) {
    if (!current_frame_ || !result.reference_frame ||
        result.map_points.size() != result.match_indices_ref.size() ||
        result.map_points.size() != result.match_indices_cur.size()) {
        return;
    }

    // The initializer estimates the current-frame pose; persist it before
    // promoting that frame and returning it from Track().
    current_frame_->SetPose(result.Tcw);

    // Promote the exact frames that were matched during initialization.
    // Re-extracting kf1 from the current image changes its feature indices and
    // makes result.match_indices_ref refer to unrelated keypoints.
    auto kf1 = std::make_shared<KeyFrame>(*result.reference_frame);
    kf1->SetPose(SE3::Identity());

    // The current frame already carries result.Tcw.
    auto kf2 = std::make_shared<KeyFrame>(*current_frame_);

    // Add keyframes to map
    map_.AddKeyFrame(kf1);
    map_.AddKeyFrame(kf2);

    // Add map points and set associations
    for (size_t i = 0; i < result.map_points.size(); ++i) {
        auto& mp = result.map_points[i];
        if (!mp) {
            continue;
        }
        int idx_ref = result.match_indices_ref[i];
        int idx_cur = result.match_indices_cur[i];

        // Insert the pre-configured MapPoint into the map
        map_.InsertMapPoint(mp);

        kf1->SetMapPointId(idx_ref, mp->Id());
        kf2->SetMapPointId(idx_cur, mp->Id());
        current_frame_->SetMapPointId(idx_cur, mp->Id());

        mp->AddObservation(kf1->Id());
        mp->AddObservation(kf2->Id());
        mp->UpdateNormal(kf1->CameraCenter());
        mp->UpdateNormal(kf2->CameraCenter());
    }

    // Notify local mapper
    if (local_mapper_) {
        local_mapper_->InsertKeyFrame(kf1);
        local_mapper_->InsertKeyFrame(kf2);
    }
    if (loop_closing_) {
        loop_closing_->InsertKeyFrame(kf1);
        loop_closing_->InsertKeyFrame(kf2);
    }

    // Initialize velocity
    velocity_ = result.Tcw;
    has_velocity_ = true;

    reference_kf_ = kf2;
}

// ── Motion model update ────────────────────────────────────────────────────

void Tracker::UpdateMotionModel() {
    if (last_frame_ && current_frame_) {
        // velocity = T_cur_w * T_w_last = T_cur_w * (T_last_w)^-1 = T_cur_w * T_w_last_inv
        velocity_ = current_frame_->Pose() * last_frame_->Pose().inverse();
        has_velocity_ = true;
    }
}

// ── Clean map points ───────────────────────────────────────────────────────

void Tracker::CleanMapPoints() {
    // Keep fresh two-view landmarks through their grace period; remove points
    // only after the map-point quality policy has enough evidence.
    auto all_mps = map_.GetAllMapPoints();
    for (const auto& mp : all_mps) {
        if (mp && mp->IsEraseReady(3)) {
            map_.EraseMapPoint(mp->Id());
        }
    }
}

void Tracker::RegisterKeyFrameObservations(const std::shared_ptr<KeyFrame>& keyframe) {
    if (!keyframe) {
        return;
    }

    const Vec3 camera_center = keyframe->CameraCenter();
    for (int i = 0; i < keyframe->NumKeyPoints(); ++i) {
        const MapPointId mp_id = keyframe->MapPointIdAt(i);
        if (mp_id.id == 0) {
            continue;
        }

        const auto mp = map_.GetMapPoint(mp_id);
        if (!mp || mp->IsBad()) {
            continue;
        }

        mp->AddObservation(keyframe->Id());
        mp->UpdateNormal(camera_center);
    }
}

// ── PnP pose estimation ────────────────────────────────────────────────────

bool Tracker::EstimatePose(const std::vector<cv::Point3f>& pts_3d,
                           const std::vector<cv::Point2f>& pts_2d, SE3& Tcw, int min_inliers) {
    if (static_cast<int>(pts_3d.size()) < min_inliers) {
        return false;
    }

    cv::Mat K = BuildK();

    geometry::PnPOptions pnp_opts;
    pnp_opts.max_reproj_error = config_.max_reprojection_error;
    pnp_opts.use_extrinsic_guess = false;

    auto result = geometry::SolvePnPRansac(pts_3d, pts_2d, K, pnp_opts);

    if (!result.valid || result.num_inliers < min_inliers) {
        return false;
    }

    // Collect inlier correspondences for refinement
    std::vector<cv::Point3f> inlier_3d;
    std::vector<cv::Point2f> inlier_2d;
    for (int idx : result.inlier_indices) {
        if (idx >= 0 && idx < static_cast<int>(pts_3d.size())) {
            inlier_3d.push_back(pts_3d[static_cast<size_t>(idx)]);
            inlier_2d.push_back(pts_2d[static_cast<size_t>(idx)]);
        }
    }

    Tcw = result.T_cw;

    // Refine
    if (inlier_3d.size() >= 6) {
        [[maybe_unused]] bool refined = geometry::RefinePnP(inlier_3d, inlier_2d, K, Tcw);
    }

    return true;
}

}  // namespace litevo::tracking
