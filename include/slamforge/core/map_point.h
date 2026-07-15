// =============================================================================
// MapPoint — 3D landmark observed by multiple frames
// =============================================================================

#pragma once

#include <Eigen/Core>

#include <opencv2/core/mat.hpp>

#include <atomic>
#include <mutex>
#include <set>
#include <vector>

#include "slamforge/core/types.h"

namespace slamforge {

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

    Vec3 Normal() const;

    /// @brief Recompute the normal as the average of all viewing directions.
    void UpdateNormal(const Vec3& camera_center);

    // ── Descriptor ─────────────────────────────────────────────────────────

    /// @brief Thread-safe shallow copy of the representative descriptor.
    ///
    /// cv::Mat uses reference-counted storage, so returning a copy of its
    /// header keeps the descriptor data alive while avoiding an unnecessary
    /// deep copy on every matching operation.
    cv::Mat Descriptor() const;
    void SetDescriptor(const cv::Mat& desc);

    // ── Observations ───────────────────────────────────────────────────────

    int Observations() const;
    void AddObservation(FrameId frame_id);
    void EraseObservation(FrameId frame_id);

    /// @brief Reference frame that created this point.
    FrameId ReferenceFrame() const;

    // ── Tracking bookkeeping ───────────────────────────────────────────────

    /// @brief Mark as observed in the current frame's local map search.
    void SetObserved(bool val);
    bool IsObserved() const;

    /// @brief Mark observed exactly once for the current frame.
    /// @return true when this call changed the flag from false to true.
    bool MarkObserved();

    /// @brief Mark as found (matched) in the current frame.
    void SetFound(bool val);
    bool IsFound() const;

    int FoundCount() const;
    void IncreaseFound(int n = 1);

    /// @brief Clear the per-frame match flag without discarding quality history.
    ///
    /// The old implementation reset @c found_count_ here once per input frame,
    /// which made the found-ratio culling policy meaningless.
    void ResetFound();

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
    void SetReferenceFrame(FrameId ref_frame);

    // ── Quality ────────────────────────────────────────────────────────────

    /// @brief Whether the point has been explicitly invalidated.
    ///
    /// A fresh monocular landmark is initially observed by exactly two
    /// keyframes.  Treating fewer than three observations as "bad" makes every
    /// valid initialization point unusable before it can be tracked again.
    /// Low-observation points are instead handled by IsEraseReady after a
    /// grace period and quality checks.
    bool IsBad() const;
    void SetBad(bool bad = true);

    /// @brief Fraction of frames where this map point was found vs predicted.
    /// Used by LocalMapper for culling decisions.
    /// @return Ratio in [0, 1], or -1 if no predictions yet.
    float GetFoundRatio() const;

    /// @brief Whether this MP is ready to be erased (bad + no recent finds).
    bool IsEraseReady(int min_observations = 3) const;

    /// @brief Track how many times it was predicted to be visible.
    void IncreaseVisible(int n = 1);
    int VisibleCount() const;

    /// @brief Frames since creation (for recent-point culling policy).
    int FramesSinceCreation() const;
    void IncrementFrame();

    // ── Scale prediction ───────────────────────────────────────────────────

    /// @brief Predict the pyramid scale at which this point would be visible.
    /// @param current_dist  Distance from the current camera to the point.
    /// @param num_levels    Number of pyramid levels.
    /// @param scale_factor  Scale factor between levels.
    int PredictScale(float current_dist, int num_levels, float scale_factor,
                     float log_scale_factor) const;

    // ── Static ─────────────────────────────────────────────────────────────

    /// @brief Reset IDs while preserving 0 as the invalid MapPointId sentinel.
    static void ResetIdCounter() { next_id_.store(1, std::memory_order_relaxed); }

private:
    MapPointId id_;
    Vec3 position_;
    Vec3 normal_{0, 0, 0};
    cv::Mat descriptor_;
    FrameId reference_frame_;

    int observations_ = 0;
    std::set<FrameId> observed_by_;
    bool is_bad_ = false;

    // Tracking bookkeeping
    bool is_observed_ = false;
    bool is_found_ = false;
    int found_count_ = 0;
    int visible_count_ = 0;
    int frames_since_creation_ = 0;

    // Thread safety
    mutable std::mutex pos_mutex_;
    mutable std::mutex data_mutex_;

    // Distance parameters for scale prediction
    float max_distance_ = 0;
    float min_distance_ = 0;

    static std::atomic<uint64_t> next_id_;
};

}  // namespace slamforge
