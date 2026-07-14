// =============================================================================
// KeyFrame — frame promoted to map landmark with covisibility graph
// =============================================================================

#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "litevo/core/frame.h"

namespace litevo {

class MapPoint;

/// @brief A keyframe in the SLAM map, extending Frame with a covisibility
///        graph, spanning tree, and lifecycle management.
///
/// KeyFrame inherits from Frame so it can be used wherever a Frame is
/// expected. The covisibility graph tracks how many MapPoints are shared
/// between pairs of keyframes, forming the basis for local mapping and
/// loop closing.
class KeyFrame : public Frame {
public:
    /// @brief Construct a keyframe by copying an existing Frame.
    explicit KeyFrame(const Frame& frame);

    /// @brief Construct a keyframe directly from an image.
    KeyFrame(const cv::Mat& image, double timestamp,
             features::OrbExtractor& extractor, const Camera& camera);

    // ── Covisibility Graph ─────────────────────────────────────────────────

    /// @brief Add or update a connection weight between this KF and another.
    /// Higher weight = more shared MapPoints.
    void AddConnection(KeyFrame* kf, int weight);

    /// @brief Remove a connection.
    void EraseConnection(KeyFrame* kf);

    /// @brief Rebuild all connections by counting shared MapPoints across
    ///        all keyframes in the map.
    /// @param all_kfs  All keyframes in the map.
    void UpdateConnections(const std::vector<std::shared_ptr<KeyFrame>>& all_kfs);

    /// @brief Get the N best covisible keyframes, sorted by weight descending.
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(int N) const;

    /// @brief Get keyframes sharing at least `min_weight` MapPoints.
    std::vector<KeyFrame*> GetCovisiblesByWeight(int min_weight) const;

    /// @brief Get the connection weight to a specific KF (0 if not connected).
    int GetWeight(KeyFrame* kf) const;

    /// @brief Raw sorted connections map.
    const std::map<KeyFrame*, int>& GetCovisibleKeyFrames() const {
        return connected_keyframes_;
    }

    // ── Spanning Tree ──────────────────────────────────────────────────────

    void SetParent(KeyFrame* parent);
    KeyFrame* GetParent() const { return parent_; }

    void AddChild(KeyFrame* child);
    void EraseChild(KeyFrame* child);
    const std::set<KeyFrame*>& GetChildren() const { return children_; }
    bool HasChild(KeyFrame* kf) const;

    // ── Status ─────────────────────────────────────────────────────────────

    void SetBad(bool flag);
    bool IsBad() const { return is_bad_; }

    // ── MapPoint helpers ───────────────────────────────────────────────────

    /// @brief Get shared_ptr to all MapPoints observed by this keyframe.
    /// @param map  The Map to look up MapPoints from.
    std::vector<std::shared_ptr<MapPoint>> GetMapPointMatches(class Map& map) const;

    // ── Mutex ──────────────────────────────────────────────────────────────

    static std::mutex& GlobalMutex() { return global_mutex_; }

private:
    /// @brief Comparator for sorting connections by weight.
    static bool CompareByWeight(
        const std::pair<KeyFrame*, int>& a,
        const std::pair<KeyFrame*, int>& b);

    std::map<KeyFrame*, int> connected_keyframes_;  // sorted by KF pointer
    std::set<KeyFrame*> children_;
    KeyFrame* parent_ = nullptr;
    bool is_bad_ = false;
    static std::mutex global_mutex_;
};

}  // namespace litevo
