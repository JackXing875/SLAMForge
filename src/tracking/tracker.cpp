// =============================================================================
// Tracker implementation
// =============================================================================

#include "slamforge/tracking/tracker.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <unordered_set>

#include "slamforge/core/camera.h"
#include "slamforge/core/frame.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map_point.h"
#include "slamforge/features/orb_extractor.h"
#include "slamforge/geometry/pnp.h"
#include "slamforge/geometry/se3.h"
#include "slamforge/loop_closing/loop_closing.h"
#include "slamforge/mapping/local_mapper.h"
#include "slamforge/tracking/initializer.h"
#include "slamforge/tracking/matcher.h"

namespace slamforge::tracking {

namespace {

struct FlowTracks {
    std::vector<cv::Point2f> previous;
    std::vector<cv::Point2f> current;
    std::vector<int> source_indices;
};

FlowTracks ForwardBackwardFlow(const cv::Mat& previous_image, const cv::Mat& current_image,
                               const std::vector<cv::Point2f>& previous_points) {
    FlowTracks tracks;
    if (previous_points.empty() || previous_image.empty() || current_image.empty()) {
        return tracks;
    }

    std::vector<cv::Point2f> current_points;
    std::vector<cv::Point2f> backward_points;
    std::vector<uchar> forward_status;
    std::vector<uchar> backward_status;
    std::vector<float> forward_error;
    std::vector<float> backward_error;
    const cv::Size window(21, 21);
    const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);

    cv::calcOpticalFlowPyrLK(previous_image, current_image, previous_points, current_points,
                             forward_status, forward_error, window, 3, criteria, 0, 1e-4);
    cv::calcOpticalFlowPyrLK(current_image, previous_image, current_points, backward_points,
                             backward_status, backward_error, window, 3, criteria, 0, 1e-4);

    tracks.previous.reserve(previous_points.size());
    tracks.current.reserve(previous_points.size());
    tracks.source_indices.reserve(previous_points.size());
    for (size_t i = 0; i < previous_points.size(); ++i) {
        if (!forward_status[i] || !backward_status[i] || forward_error[i] > 30.0f ||
            cv::norm(previous_points[i] - backward_points[i]) > 1.5) {
            continue;
        }
        const cv::Point2f& point = current_points[i];
        const float image_width = static_cast<float>(current_image.cols);
        const float image_height = static_cast<float>(current_image.rows);
        if (point.x < 4.0f || point.y < 4.0f || point.x >= image_width - 4.0f ||
            point.y >= image_height - 4.0f) {
            continue;
        }
        tracks.previous.push_back(previous_points[i]);
        tracks.current.push_back(point);
        tracks.source_indices.push_back(static_cast<int>(i));
    }
    return tracks;
}

cv::Point2f UndistortedPixel(const Camera& camera, const cv::Point2f& pixel) {
    if (!camera.has_distortion()) {
        return pixel;
    }
    const Vec3 ray = camera.Unproject(Vec2(pixel.x, pixel.y));
    return cv::Point2f(static_cast<float>(camera.fx() * ray.x() / ray.z() + camera.cx()),
                       static_cast<float>(camera.fy() * ray.y() / ray.z() + camera.cy()));
}

}  // namespace

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
    used_relative_motion_ = false;
    initializer_->Reset();
    Frame::ResetIdCounter();
    MapPoint::ResetIdCounter();
}

std::optional<SE3> Tracker::SynchronizeCurrentPoseFromMap() {
    if (!current_frame_) {
        return std::nullopt;
    }
    auto graph_lock = map_.AcquireGraphLock();
    const auto keyframe = map_.GetKeyFrame(current_frame_->Id());
    if (!keyframe || keyframe->IsBad()) {
        return current_frame_->Pose();
    }

    const SE3 optimized_pose = keyframe->Pose();
    current_frame_->SetPose(optimized_pose);
    if (last_frame_ && last_frame_->Id() == current_frame_->Id()) {
        last_frame_->SetPose(optimized_pose);
    }
    // Local BA changed the keyframe after the old velocity was computed.
    // Reacquire landmarks on the next frame instead of extrapolating between
    // two different map states.
    has_velocity_ = false;
    velocity_ = SE3::Identity();
    return optimized_pose;
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
    used_relative_motion_ = false;

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

        // Stage 2: KLT flow preserves landmark tracks through blur and
        // descriptor instability.
        if (!ok) {
            ok = TrackWithOpticalFlow();
        }

        // Stage 3: Reference keyframe fallback
        if (!ok) {
            ok = TrackReferenceKeyFrame();
        }

        // Stage 4: Relocalization
        if (!ok) {
            ok = Relocalization();
        }

        // Stage 5: keep odometry alive while a fresh local map is built.
        if (!ok) {
            ok = TrackRelativeMotion();
        }

        if (ok) {
            // Stage 6: Local map refinement
            TrackLocalMap();

            // Update motion model
            if (last_frame_) {
                UpdateMotionModel();
            }

            // Keyframe decision
            const bool relative_keyframe_due =
                used_relative_motion_ &&
                frames_since_last_kf_ >= std::max(2, config_.min_frames_between_kf);
            if (relative_keyframe_due || NeedNewKeyFrame()) {
                InsertCurrentKeyFrame();
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
        if (!ok) {
            ok = TrackRelativeMotion();
        }
        if (ok) {
            state_ = TrackingState::OK;
            TrackLocalMap();
            if (last_frame_) {
                UpdateMotionModel();
            }
            const bool relative_keyframe_due =
                used_relative_motion_ &&
                frames_since_last_kf_ >= std::max(2, config_.min_frames_between_kf);
            if (relative_keyframe_due || NeedNewKeyFrame()) {
                InsertCurrentKeyFrame();
            } else {
                frames_since_last_kf_++;
            }
            last_frame_ = current_frame_;
            return current_frame_->Pose();
        }
        if (last_frame_) {
            current_frame_->SetPose(last_frame_->Pose());
        }
        last_frame_ = current_frame_;
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
    std::unordered_set<uint64_t> seen_mps;

    for (int i = 0; i < last_frame_->NumKeyPoints(); ++i) {
        MapPointId mp_id = last_frame_->MapPointIdAt(i);
        if (mp_id.id != 0) {
            auto mp = map_.GetMapPoint(mp_id);
            if (mp && !mp->IsBad() && mp->Position().allFinite() &&
                seen_mps.insert(mp_id.id).second) {
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
    std::vector<int> matched_kp_indices;
    const auto& kps = current_frame_->KeyPointsUndistorted();

    for (const auto& [kp_idx, mp_id] : matches) {
        auto mp = map_.GetMapPoint(mp_id);
        if (!mp || mp->IsBad())
            continue;

        const Vec3& pw = mp->Position();
        if (!pw.allFinite())
            continue;
        pts_3d.emplace_back(static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                            static_cast<float>(pw.z()));
        pts_2d.emplace_back(kps[static_cast<size_t>(kp_idx)].pt.x,
                            kps[static_cast<size_t>(kp_idx)].pt.y);
        matched_mps.push_back(mp);
        matched_kp_indices.push_back(kp_idx);
    }

    // PnP + refine
    SE3 pose = predicted_pose;
    std::vector<int> inlier_indices;
    if (!EstimatePose(pts_3d, pts_2d, pose, 10, &inlier_indices, true)) {
        return false;
    }

    current_frame_->SetPose(pose);
    for (int inlier_idx : inlier_indices) {
        if (inlier_idx < 0 || inlier_idx >= static_cast<int>(matched_mps.size()))
            continue;
        const auto& mp = matched_mps[static_cast<size_t>(inlier_idx)];
        current_frame_->SetMapPointId(matched_kp_indices[static_cast<size_t>(inlier_idx)], mp->Id());
        mp->IncreaseFound();
        mp->SetFound(true);
    }
    num_tracked_points_ = static_cast<int>(inlier_indices.size());
    return true;
}

// ── Track last-frame landmarks with optical flow ──────────────────────────

bool Tracker::TrackWithOpticalFlow() {
    if (!last_frame_) {
        return false;
    }

    std::vector<cv::Point2f> previous_points;
    std::vector<std::shared_ptr<MapPoint>> source_map_points;
    std::unordered_set<uint64_t> seen;
    const auto& last_keypoints = last_frame_->KeyPoints();
    for (int i = 0; i < last_frame_->NumKeyPoints(); ++i) {
        const MapPointId mp_id = last_frame_->MapPointIdAt(i);
        if (mp_id.id == 0 || !seen.insert(mp_id.id).second) {
            continue;
        }
        const auto mp = map_.GetMapPoint(mp_id);
        if (!mp || mp->IsBad() || !mp->Position().allFinite()) {
            continue;
        }
        previous_points.push_back(last_keypoints[static_cast<size_t>(i)].pt);
        source_map_points.push_back(mp);
    }

    if (previous_points.size() < 12) {
        return false;
    }

    const FlowTracks tracks =
        ForwardBackwardFlow(last_frame_->Image(), current_frame_->Image(), previous_points);
    if (tracks.current.size() < 12) {
        return false;
    }

    std::vector<cv::Point3f> points_3d;
    std::vector<cv::Point2f> points_2d;
    std::vector<std::shared_ptr<MapPoint>> tracked_map_points;
    points_3d.reserve(tracks.current.size());
    points_2d.reserve(tracks.current.size());
    tracked_map_points.reserve(tracks.current.size());
    for (size_t i = 0; i < tracks.current.size(); ++i) {
        const int source_idx = tracks.source_indices[i];
        const auto& mp = source_map_points[static_cast<size_t>(source_idx)];
        const Vec3 position = mp->Position();
        points_3d.emplace_back(static_cast<float>(position.x()), static_cast<float>(position.y()),
                               static_cast<float>(position.z()));
        points_2d.push_back(UndistortedPixel(camera_, tracks.current[i]));
        tracked_map_points.push_back(mp);
    }

    SE3 pose = has_velocity_ ? velocity_ * last_frame_->Pose() : last_frame_->Pose();
    std::vector<int> inliers;
    if (!EstimatePose(points_3d, points_2d, pose, 10, &inliers, true)) {
        return false;
    }

    current_frame_->SetPose(pose);
    std::unordered_set<int> associated_keypoints;
    for (int inlier_idx : inliers) {
        if (inlier_idx < 0 || inlier_idx >= static_cast<int>(tracked_map_points.size())) {
            continue;
        }
        const auto& mp = tracked_map_points[static_cast<size_t>(inlier_idx)];
        const cv::Point2f pixel = points_2d[static_cast<size_t>(inlier_idx)];
        const auto candidates = current_frame_->GetFeaturesInArea(pixel.x, pixel.y, 4.0f);
        int best_idx = -1;
        int best_distance = 81;
        for (int candidate : candidates) {
            if (associated_keypoints.contains(candidate) ||
                current_frame_->MapPointIdAt(candidate).id != 0) {
                continue;
            }
            const int distance = FeatureMatcher::DescriptorDistance(
                mp->Descriptor(), current_frame_->Descriptors().row(candidate));
            if (distance < best_distance) {
                best_distance = distance;
                best_idx = candidate;
            }
        }
        if (best_idx >= 0) {
            current_frame_->SetMapPointId(best_idx, mp->Id());
            associated_keypoints.insert(best_idx);
            mp->IncreaseFound();
            mp->SetFound(true);
        }
    }
    num_tracked_points_ = static_cast<int>(inliers.size());
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
    std::vector<int> matched_kp_indices;
    const auto& kps = current_frame_->KeyPointsUndistorted();

    for (const auto& [ref_idx, cur_idx] : matches) {
        MapPointId mp_id = reference_kf_->MapPointIdAt(ref_idx);
        if (mp_id.id == 0)
            continue;

        auto mp = map_.GetMapPoint(mp_id);
        if (!mp || mp->IsBad())
            continue;

        const Vec3& pw = mp->Position();
        if (!pw.allFinite())
            continue;
        pts_3d.emplace_back(static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                            static_cast<float>(pw.z()));
        pts_2d.emplace_back(kps[static_cast<size_t>(cur_idx)].pt.x,
                            kps[static_cast<size_t>(cur_idx)].pt.y);
        matched_mps.push_back(mp);
        matched_kp_indices.push_back(cur_idx);
    }

    if (static_cast<int>(pts_3d.size()) < 10) {
        return false;
    }

    // Use last frame's pose as initial guess
    SE3 pose = last_frame_ ? last_frame_->Pose() : SE3::Identity();

    std::vector<int> inlier_indices;
    if (!EstimatePose(pts_3d, pts_2d, pose, 10, &inlier_indices, true)) {
        return false;
    }

    current_frame_->SetPose(pose);
    for (int inlier_idx : inlier_indices) {
        if (inlier_idx < 0 || inlier_idx >= static_cast<int>(matched_mps.size()))
            continue;
        const auto& mp = matched_mps[static_cast<size_t>(inlier_idx)];
        current_frame_->SetMapPointId(matched_kp_indices[static_cast<size_t>(inlier_idx)], mp->Id());
        mp->IncreaseVisible();
        mp->IncreaseFound();
        mp->SetFound(true);
    }
    num_tracked_points_ = static_cast<int>(inlier_indices.size());
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
        std::vector<int> correspondence_kp_indices;
        std::vector<std::shared_ptr<MapPoint>> correspondence_mps;
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
            if (!pw.allFinite())
                continue;
            pts_3d.emplace_back(static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                                static_cast<float>(pw.z()));
            pts_2d.emplace_back(kps[static_cast<size_t>(i)].pt.x, kps[static_cast<size_t>(i)].pt.y);
            correspondence_kp_indices.push_back(i);
            correspondence_mps.push_back(mp);
        }

        // Add new matches from local map
        for (const auto& [kp_idx, mp_id] : matches) {
            auto mp = map_.GetMapPoint(mp_id);
            if (!mp || mp->IsBad())
                continue;

            const Vec3& pw = mp->Position();
            if (!pw.allFinite())
                continue;
            pts_3d.emplace_back(static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                                static_cast<float>(pw.z()));
            pts_2d.emplace_back(kps[static_cast<size_t>(kp_idx)].pt.x,
                                kps[static_cast<size_t>(kp_idx)].pt.y);

            correspondence_kp_indices.push_back(kp_idx);
            correspondence_mps.push_back(mp);
        }

        // Refine pose with all correspondences
        if (pts_3d.size() >= 10) {
            SE3 pose = current_frame_->Pose();
            std::vector<int> inlier_indices;
            if (EstimatePose(pts_3d, pts_2d, pose, 10, &inlier_indices, true)) {
                current_frame_->SetPose(pose);
                for (int kp_idx : correspondence_kp_indices) {
                    current_frame_->SetMapPointId(kp_idx, MapPointId{0});
                }
                for (int inlier_idx : inlier_indices) {
                    if (inlier_idx < 0 ||
                        inlier_idx >= static_cast<int>(correspondence_mps.size())) {
                        continue;
                    }
                    const auto& mp = correspondence_mps[static_cast<size_t>(inlier_idx)];
                    current_frame_->SetMapPointId(
                        correspondence_kp_indices[static_cast<size_t>(inlier_idx)], mp->Id());
                    mp->IncreaseFound();
                    mp->SetFound(true);
                }
                num_tracked_points_ = static_cast<int>(inlier_indices.size());
            }
        }
    }

    return true;
}

// ── Frame-to-frame geometric fallback ─────────────────────────────────────

bool Tracker::TrackRelativeMotion() {
    if (!last_frame_ || last_frame_->NumKeyPoints() < 30) {
        return false;
    }

    std::vector<cv::Point2f> previous_points;
    previous_points.reserve(static_cast<size_t>(last_frame_->NumKeyPoints()));
    for (const auto& keypoint : last_frame_->KeyPoints()) {
        previous_points.push_back(keypoint.pt);
    }

    const FlowTracks tracks =
        ForwardBackwardFlow(last_frame_->Image(), current_frame_->Image(), previous_points);
    if (tracks.current.size() < 30) {
        return false;
    }

    std::vector<cv::Point2f> previous_undistorted;
    std::vector<cv::Point2f> current_undistorted;
    previous_undistorted.reserve(tracks.current.size());
    current_undistorted.reserve(tracks.current.size());
    for (size_t i = 0; i < tracks.current.size(); ++i) {
        previous_undistorted.push_back(UndistortedPixel(camera_, tracks.previous[i]));
        current_undistorted.push_back(UndistortedPixel(camera_, tracks.current[i]));
    }

    cv::Mat inlier_mask;
    const cv::Mat essential =
        cv::findEssentialMat(previous_undistorted, current_undistorted, BuildK(), cv::RANSAC,
                             0.999, 1.5, inlier_mask);
    if (essential.empty()) {
        return false;
    }

    cv::Mat rotation;
    cv::Mat unit_translation;
    const int inlier_count = cv::recoverPose(essential, previous_undistorted, current_undistorted,
                                             BuildK(), rotation, unit_translation, inlier_mask);
    if (inlier_count < 25) {
        return false;
    }

    cv::Mat rotation_vector;
    cv::Rodrigues(rotation, rotation_vector);
    SE3 relative_pose = geometry::FromOpenCVRt(rotation_vector, unit_translation);
    double translation_scale = velocity_.translation().norm();
    if (!std::isfinite(translation_scale) || translation_scale < 1e-4) {
        translation_scale = 0.003;
    }
    translation_scale = std::clamp(translation_scale, 0.0005, 0.05);
    relative_pose.translation() *= translation_scale;
    current_frame_->SetPose(relative_pose * last_frame_->Pose());

    // Preserve any last-frame landmark tracks that also agree with essential
    // geometry.  A local-map projection pass will add more associations.
    std::unordered_set<int> associated_keypoints;
    for (size_t i = 0; i < tracks.current.size(); ++i) {
        if (inlier_mask.at<uchar>(static_cast<int>(i)) == 0) {
            continue;
        }
        const int last_idx = tracks.source_indices[i];
        const MapPointId mp_id = last_frame_->MapPointIdAt(last_idx);
        if (mp_id.id == 0) {
            continue;
        }
        const auto mp = map_.GetMapPoint(mp_id);
        if (!mp || mp->IsBad() || !mp->Position().allFinite()) {
            continue;
        }

        const cv::Point2f pixel = current_undistorted[i];
        const auto candidates = current_frame_->GetFeaturesInArea(pixel.x, pixel.y, 5.0f);
        int best_idx = -1;
        int best_distance = 91;
        for (int candidate : candidates) {
            if (associated_keypoints.contains(candidate) ||
                current_frame_->MapPointIdAt(candidate).id != 0) {
                continue;
            }
            const int distance = FeatureMatcher::DescriptorDistance(
                mp->Descriptor(), current_frame_->Descriptors().row(candidate));
            if (distance < best_distance) {
                best_distance = distance;
                best_idx = candidate;
            }
        }
        if (best_idx >= 0) {
            current_frame_->SetMapPointId(best_idx, mp_id);
            associated_keypoints.insert(best_idx);
            mp->IncreaseFound();
            mp->SetFound(true);
        }
    }

    num_tracked_points_ = inlier_count;
    used_relative_motion_ = true;
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
        std::vector<int> inlier_indices;
        if (EstimatePose(pts_3d, pts_2d, pose, 15, &inlier_indices, false)) {
            current_frame_->SetPose(pose);
            for (int inlier_idx : inlier_indices) {
                if (inlier_idx < 0 || inlier_idx >= static_cast<int>(matched_mps.size()))
                    continue;
                const auto& [cur_idx, mp] = matched_mps[static_cast<size_t>(inlier_idx)];
                current_frame_->SetMapPointId(cur_idx, mp->Id());
                mp->IncreaseVisible();
                mp->IncreaseFound();
                mp->SetFound(true);
            }
            num_tracked_points_ = static_cast<int>(inlier_indices.size());
            return true;
        }
    }

    return false;
}

void Tracker::InsertCurrentKeyFrame() {
    if (!current_frame_) {
        return;
    }
    auto kf = std::make_shared<KeyFrame>(*current_frame_);
    map_.AddKeyFrame(kf);
    RegisterKeyFrameObservations(kf);
    reference_kf_ = kf;

    if (local_mapper_) {
        local_mapper_->InsertKeyFrame(kf);
    }
    if (loop_closing_) {
        loop_closing_->InsertKeyFrame(kf);
    }

    frames_since_last_kf_ = 0;
    is_new_keyframe_ = true;
}

// ── Keyframe decision ──────────────────────────────────────────────────────

bool Tracker::NeedNewKeyFrame() {
    if (num_tracked_points_ < 15) {
        return false;
    }

    int ref_map_points = reference_kf_ ? reference_kf_->NumMapPoints() : num_tracked_points_;
    if (ref_map_points == 0)
        ref_map_points = 1;

    // Insert only after the configured minimum spacing.  The old `c1 || c2 ||
    // c3` expression allowed c1 to bypass this gate and produced a keyframe
    // every 3-4 frames on sequence_01.
    const bool max_spacing_reached = frames_since_last_kf_ >= config_.max_frames_between_kf;
    if (frames_since_last_kf_ < config_.min_frames_between_kf && !max_spacing_reached) {
        return false;
    }

    // A 75% reference ratio retains enough temporal baseline without flooding
    // the mapper whenever a handful of features changes.
    const bool tracking_weakened =
        num_tracked_points_ < static_cast<int>(static_cast<double>(ref_map_points) * 0.75);

    // Condition 2: max frames exceeded
    // Let the mapper catch up unless the maximum temporal spacing forces a KF.
    if (local_mapper_ && local_mapper_->QueueSize() > 2 && !max_spacing_reached) {
        return false;
    }

    return tracking_weakened || max_spacing_reached;
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
                           const std::vector<cv::Point2f>& pts_2d, SE3& Tcw, int min_inliers,
                           std::vector<int>* inlier_indices, bool use_pose_guess) {
    if (static_cast<int>(pts_3d.size()) < min_inliers) {
        return false;
    }

    cv::Mat K = BuildK();

    geometry::PnPOptions pnp_opts;
    pnp_opts.max_reproj_error = config_.max_reprojection_error;
    // RANSAC must remain free to find the geometrically best basin.  The
    // motion model is used below as a continuity validator, not as an
    // optimization prior that can pin ITERATIVE PnP to a stale pose.
    pnp_opts.use_extrinsic_guess = false;

    const SE3 pose_guess = Tcw;

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

    // Refine
    SE3 refined_pose = result.T_cw;
    if (inlier_3d.size() >= 6) {
        [[maybe_unused]] const bool refined =
            geometry::RefinePnP(inlier_3d, inlier_2d, K, refined_pose);
    }

    if (!refined_pose.matrix().allFinite() ||
        std::abs(refined_pose.rotation().determinant() - 1.0) > 1e-3) {
        return false;
    }

    // Re-check every RANSAC inlier after nonlinear refinement.  This catches
    // the rare OpenCV PnP solutions that report inliers but place the camera
    // thousands of map units away.
    std::vector<int> verified_inliers;
    verified_inliers.reserve(result.inlier_indices.size());
    const double verification_threshold = config_.max_reprojection_error * 1.5;
    const double verification_threshold_sq = verification_threshold * verification_threshold;
    for (int idx : result.inlier_indices) {
        if (idx < 0 || idx >= static_cast<int>(pts_3d.size())) {
            continue;
        }
        const cv::Point3f& point = pts_3d[static_cast<size_t>(idx)];
        const Vec3 point_world(point.x, point.y, point.z);
        const Vec3 point_camera = refined_pose * point_world;
        if (!point_camera.allFinite() || point_camera.z() <= 1e-6) {
            continue;
        }
        const Vec2 projected = camera_.ProjectUndistorted(point_camera);
        const cv::Point2f& observed = pts_2d[static_cast<size_t>(idx)];
        const double dx = projected.x() - static_cast<double>(observed.x);
        const double dy = projected.y() - static_cast<double>(observed.y);
        if (dx * dx + dy * dy <= verification_threshold_sq) {
            verified_inliers.push_back(idx);
        }
    }

    if (static_cast<int>(verified_inliers.size()) < min_inliers ||
        verified_inliers.size() * 10 < result.inlier_indices.size() * 7) {
        return false;
    }

    if (use_pose_guess) {
        const Vec3 refined_center =
            -refined_pose.rotation().transpose() * refined_pose.translation();
        const Vec3 guessed_center = -pose_guess.rotation().transpose() * pose_guess.translation();
        const double expected_step = std::max(velocity_.translation().norm(), 1e-4);
        const double max_center_jump = std::clamp(expected_step * 15.0, 0.15, 1.0);
        if ((refined_center - guessed_center).norm() > max_center_jump) {
            return false;
        }

        const Mat3 rotation_delta = refined_pose.rotation() * pose_guess.rotation().transpose();
        const double cosine =
            std::clamp((rotation_delta.trace() - 1.0) * 0.5, -1.0, 1.0);
        if (std::acos(cosine) > 25.0 * std::numbers::pi_v<double> / 180.0) {
            return false;
        }
    }

    SE3 published_pose = refined_pose;
    if (use_pose_guess) {
        // Independent PnP is accurate but exhibits high-frequency monocular
        // translation jitter.  Fuse it with the constant-velocity prediction
        // only after geometric validation, so the prediction cannot trap the
        // RANSAC solve in a wrong basin.
        const SE3 world_from_guess = pose_guess.inverse();
        const SE3 world_from_measurement = refined_pose.inverse();
        Eigen::Quaterniond guess_rotation(world_from_guess.rotation());
        Eigen::Quaterniond measured_rotation(world_from_measurement.rotation());
        if (guess_rotation.dot(measured_rotation) < 0.0) {
            measured_rotation.coeffs() *= -1.0;
        }

        SE3 world_from_blended = SE3::Identity();
        world_from_blended.linear() = guess_rotation.slerp(0.5, measured_rotation).toRotationMatrix();
        world_from_blended.translation() =
            0.5 * world_from_guess.translation() + 0.5 * world_from_measurement.translation();
        published_pose = world_from_blended.inverse();
    }

    Tcw = published_pose;
    if (inlier_indices) {
        *inlier_indices = std::move(verified_inliers);
    }

    return true;
}

}  // namespace slamforge::tracking
