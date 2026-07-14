// =============================================================================
// MapPoint — 3D landmark observed by multiple frames
// =============================================================================

#pragma once

#include <Eigen/Core>

#include <opencv2/core/mat.hpp>

#include <mutex>
#include <set>
#include <vector>

#include "litevo/core/types.h"

namespace litevo {

/// @brief A 3D landmark point in the world, created by triangulating
///        corresponding features across multiple views.
///
/// Each MapPoint stores its 3D position, a representative descriptor
/// for matching, and bookkeeping information used during tracking.
class MapPoint {
public:
    /// @brief Construct a MapPoint at a given world position.
    /// @param position         3D world coordinates.
    /// @param reference_frame  The frame that created this point.
    MapPoint(const Vec3& position, FrameId reference_frame);

    // ── Identity ───────────────────────────────────────────────────────────

    MapPointId Id() const { return id_; }

    // ── Position ───────────────────────────────────────────────────────────

    /// @brief Thread-safe position read.
    Vec3 Position() const {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        return position_;
    }

    /// @brief Thread-safe position write.
    void SetPosition(const Vec3& pos) {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        position_ = pos;
    }

    // ── Normal (average viewing direction) ─────────────────────────────────

    const Vec3& Normal() const { return normal_; }

    /// @brief Recompute the normal as the average of all viewing directions.
    void UpdateNormal(const Vec3& camera_center);

    // ── Descriptor ─────────────────────────────────────────────────────────

    const cv::Mat& Descriptor() const { return descriptor_; }
    void SetDescriptor(const cv::Mat& desc);

    // ── Observations ───────────────────────────────────────────────────────

    int Observations() const { return observations_; }
    void AddObservation(FrameId frame_id);
    void EraseObservation(FrameId frame_id);

    /// @brief Reference frame that created this point.
    FrameId ReferenceFrame() const { return reference_frame_; }

    // ── Tracking bookkeeping ───────────────────────────────────────────────

    /// @brief Mark as observed in the current frame's local map search.
    void SetObserved(bool val) { is_observed_ = val; }
    bool IsObserved() const { return is_observed_; }

    /// @brief Mark as found (matched) in the current frame.
    void SetFound(bool val) { is_found_ = val; }
    bool IsFound() const { return is_found_; }

    int FoundCount() const { return found_count_; }
    void IncreaseFound(int n = 1) { found_count_ += n; }
    void ResetFound() { found_count_ = 0; }

    // ── Fusion ───────────────────────────────────────────────────────────────

    /// @brief Replace this MapPoint with another (fusion after loop closure).
    ///
    /// Transfers all observations from `other` to `this`, updates the
    /// representative descriptor, copies the position if `other` has more
    /// observations, and marks `other` as effectively bad.
    ///
    /// @param other  The MapPoint to absorb. Will NOT be erased from Map.
    void Replace(MapPoint* other);

    /// @brief Set the reference frame (used after loop correction).
    void SetReferenceFrame(FrameId ref_frame) { reference_frame_ = ref_frame; }

    // ── Quality ────────────────────────────────────────────────────────────

    /// @brief Whether the point should be culled (too few observations).
    bool IsBad(int min_observations = 3) const { return observations_ < min_observations; }

    /// @brief Fraction of frames where this map point was found vs predicted.
    /// Used by LocalMapper for culling decisions.
    /// @return Ratio in [0, 1], or -1 if no predictions yet.
    float GetFoundRatio() const;

    /// @brief Whether this MP is ready to be erased (bad + no recent finds).
    bool IsEraseReady(int min_observations = 3) const;

    /// @brief Track how many times it was predicted to be visible.
    void IncreaseVisible(int n = 1) { visible_count_ += n; }
    int VisibleCount() const { return visible_count_; }

    /// @brief Frames since creation (for recent-point culling policy).
    int FramesSinceCreation() const { return frames_since_creation_; }
    void IncrementFrame() { frames_since_creation_++; }

    // ── Scale prediction ───────────────────────────────────────────────────

    /// @brief Predict the pyramid scale at which this point would be visible.
    /// @param current_dist  Distance from the current camera to the point.
    /// @param num_levels    Number of pyramid levels.
    /// @param scale_factor  Scale factor between levels.
    int PredictScale(float current_dist, int num_levels, float scale_factor,
                     float log_scale_factor) const;

    // ── Static ─────────────────────────────────────────────────────────────

    static void ResetIdCounter() { next_id_ = 0; }

private:
    MapPointId id_;
    Vec3 position_;
    Vec3 normal_{0, 0, 0};
    cv::Mat descriptor_;
    FrameId reference_frame_;

    int observations_ = 0;
    std::set<FrameId> observed_by_;

    // Tracking bookkeeping
    bool is_observed_ = false;
    bool is_found_ = false;
    int found_count_ = 0;
    int visible_count_ = 0;
    int frames_since_creation_ = 0;

    // Thread safety
    mutable std::mutex pos_mutex_;

    // Distance parameters for scale prediction
    float max_distance_ = 0;
    float min_distance_ = 0;

    static uint64_t next_id_;
};

}  // namespace litevo
