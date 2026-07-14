// =============================================================================
// Frame — fundamental per-image data unit
// =============================================================================

#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>

#include <memory>
#include <vector>

#include "litevo/core/camera.h"
#include "litevo/core/feature_grid.h"
#include "litevo/core/types.h"

namespace litevo::features {
class OrbExtractor;
}

namespace litevo {

/// @brief A single camera frame with extracted features, pose, and map point
///        associations.
///
/// Each Frame owns its image, keypoints, descriptors, and a spatial grid
/// for fast feature lookup. Frames are created by the Tracker and may be
/// promoted to keyframes for insertion into the Map.
class Frame {
public:
    /// @brief Construct a frame from an image.
    ///
    /// Extracts ORB features, undistorts keypoints, and builds the
    /// feature grid for spatial queries.
    ///
    /// @param image      Grayscale input image (CV_8UC1).
    /// @param timestamp  Time in seconds.
    /// @param extractor  ORB feature extractor.
    /// @param camera     Camera model for undistortion and projection.
    Frame(const cv::Mat& image, double timestamp, features::OrbExtractor& extractor,
          const Camera& camera);

    // ── Identity ───────────────────────────────────────────────────────────

    /// @brief Unique frame identifier.
    FrameId Id() const { return id_; }

    /// @brief Timestamp in seconds.
    double Timestamp() const { return timestamp_; }

    // ── Image ──────────────────────────────────────────────────────────────

    /// @brief Grayscale image.
    const cv::Mat& Image() const { return image_; }

    // ── Pose ───────────────────────────────────────────────────────────────

    /// @brief Camera pose (world from camera, Tcw).
    const SE3& Pose() const { return Tcw_; }

    /// @brief Set the camera pose.
    void SetPose(const SE3& Tcw) { Tcw_ = Tcw; }

    /// @brief Camera center in world coordinates.
    Vec3 CameraCenter() const;

    // ── Features ───────────────────────────────────────────────────────────

    /// @brief Detected keypoints (distorted pixel coordinates).
    const std::vector<cv::KeyPoint>& KeyPoints() const { return keypoints_; }

    /// @brief Undistorted keypoints.
    const std::vector<cv::KeyPoint>& KeyPointsUndistorted() const { return keypoints_undistorted_; }

    /// @brief ORB descriptors, one row per keypoint.
    const cv::Mat& Descriptors() const { return descriptors_; }

    /// @brief Number of keypoints.
    int NumKeyPoints() const { return static_cast<int>(keypoints_.size()); }

    // ── MapPoint associations ──────────────────────────────────────────────

    /// @brief Get the MapPoint ID associated with a keypoint index.
    /// @return MapPointId with id=0 if no association (null).
    MapPointId MapPointIdAt(int idx) const;

    /// @brief Set the MapPoint ID for a keypoint index.
    void SetMapPointId(int idx, MapPointId mp_id);

    /// @brief Count of keypoints with valid MapPoint associations.
    int NumMapPoints() const;

    /// @brief Get indices of all keypoints that have valid MapPoint associations.
    std::vector<int> GetMapPointIndices() const;

    // ── Spatial search ─────────────────────────────────────────────────────

    /// @brief Get keypoint indices within a search window.
    /// @param x, y      Search center (undistorted coordinates).
    /// @param radius     Search radius in pixels.
    /// @param min_level  Minimum pyramid level (-1 = all).
    /// @param max_level  Maximum pyramid level (-1 = all).
    std::vector<int> GetFeaturesInArea(float x, float y, float radius, int min_level = -1,
                                       int max_level = -1) const;

    // ── Keyframe flag ──────────────────────────────────────────────────────

    bool IsKeyFrame() const { return is_keyframe_; }
    void SetKeyFrame(bool is_kf) { is_keyframe_ = is_kf; }

    // ── Camera ─────────────────────────────────────────────────────────────

    const Camera& GetCamera() const { return camera_; }

    // ── Static ID management ───────────────────────────────────────────────

    /// @brief Reset the global frame ID counter (e.g., for testing).
    static void ResetIdCounter() { next_id_ = 0; }

private:
    void ExtractFeatures(features::OrbExtractor& extractor);
    void UndistortKeyPoints();
    void AssignFeaturesToGrid();

    FrameId id_;
    double timestamp_;
    cv::Mat image_;
    SE3 Tcw_{SE3::Identity()};
    Camera camera_;

    std::vector<cv::KeyPoint> keypoints_;
    std::vector<cv::KeyPoint> keypoints_undistorted_;
    cv::Mat descriptors_;

    std::vector<MapPointId> map_point_ids_;
    FeatureGrid grid_{};

    bool is_keyframe_ = false;

    static uint64_t next_id_;
};

}  // namespace litevo
