// =============================================================================
// LoopVerifier — geometric verification of loop candidates via Sim(3)
// =============================================================================
// For a candidate loop pair (current KF, matched KF):
//   1. Match features using BoW-accelerated search
//   2. Triangulate matches to get 3D-3D correspondences
//   3. Estimate Sim(3) via RANSAC + Umeyama
//   4. Check inlier count and reprojection error
//   5. Bidirectional verification

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "slamforge/core/types.h"
#include "slamforge/geometry/sim3.h"

namespace slamforge {

class Camera;
class KeyFrame;
class Map;
class MapPoint;
struct MapPointId;

namespace loop_closing {

class LoopDetector;

/// @brief Result of geometric loop verification.
struct VerificationResult {
    bool valid = false;
    geometry::Sim3 S_cw;                       ///< Sim(3) correction for current KF
    geometry::Sim3Mat Mg2o;                    ///< Accumulated correction for g2o
    std::vector<std::pair<int, int>> matches;  ///< 2D-2D matches (idx_cur, idx_cand)
    std::vector<MapPointId> matched_mps_cur;   ///< Matched MPs in current KF
    std::vector<MapPointId> matched_mps_cand;  ///< Matched MPs in candidate KF
    int num_inliers = 0;
    int num_initial_matches = 0;
    int num_geometric_matches = 0;
    int num_3d_correspondences = 0;
    int num_sim3_inliers = 0;
    int num_reprojection_inliers = 0;
    double estimated_scale = 1.0;
};

/// @brief Configuration for LoopVerifier.
struct LoopVerifierConfig {
    int min_inliers = 6;              ///< Min inliers for valid loop
    double max_reproj_error = 8.0;    ///< Max bidirectional reprojection error (pixels)
    int min_boW_matches = 20;         ///< Min descriptor matches to attempt Sim3
    int ransac_iterations = 300;      ///< RANSAC iterations for Sim3
    double sim3_max_error_3d = 0.1;   ///< Minimum scale-adaptive 3D inlier gate
    double descriptor_ratio = 0.75;   ///< Lowe ratio for ORB descriptor matching
};

/// @brief Geometrically verifies loop candidates using Sim(3) estimation.
class LoopVerifier {
public:
    explicit LoopVerifier(const LoopVerifierConfig& config = {}, const Camera* camera = nullptr);

    /// @brief Set the camera model (needed for triangulation/projection).
    void SetCamera(const Camera* camera) { camera_ = camera; }

    /// @brief Verify a loop candidate pair geometrically.
    ///
    /// Steps:
    ///   1. BoW-guided descriptor matching
    ///   2. For matched features with MapPoints, get 3D-3D correspondences
    ///   3. RANSAC Sim(3) estimation
    ///   4. Reprojection check on inliers
    ///   5. Return result with Sim(3) correction and match info
    VerificationResult Verify(std::shared_ptr<KeyFrame> current_kf,
                              std::shared_ptr<KeyFrame> candidate_kf, LoopDetector& detector,
                              Map& map);

private:
    /// @brief Search matches between two KFs using BoW word grouping.
    int SearchByBoW(std::shared_ptr<KeyFrame> kf1, std::shared_ptr<KeyFrame> kf2,
                    std::vector<std::pair<int, int>>& matches,
                    std::vector<std::pair<int, float>>& word_weights_1,
                    std::vector<std::pair<int, float>>& word_weights_2);

    /// @brief Build 3D-3D correspondences from matched features that have MapPoints.
    int Build3DCorrespondences(std::shared_ptr<KeyFrame> kf_cur, std::shared_ptr<KeyFrame> kf_cand,
                               const std::vector<std::pair<int, int>>& matches, Map& map,
                               std::vector<Vec3>& pts_cur_w, std::vector<Vec3>& pts_cand_w,
                               std::vector<int>& match_indices_cur,
                               std::vector<int>& match_indices_cand);

    LoopVerifierConfig config_;
    const Camera* camera_ = nullptr;
};

}  // namespace loop_closing
}  // namespace slamforge
