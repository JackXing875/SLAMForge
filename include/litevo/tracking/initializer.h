// =============================================================================
// MonocularInitializer — two-view map initialization
// =============================================================================

#pragma once

#include <opencv2/core/types.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "litevo/core/types.h"

namespace litevo {

class Camera;
class Frame;
class MapPoint;

namespace features {
class OrbExtractor;
}

namespace tracking {

/// @brief Result of two-view monocular initialization.
struct InitializationResult {
    bool success = false;
    SE3 Tcw;                                            ///< Pose of the second frame
    std::vector<std::shared_ptr<MapPoint>> map_points;  ///< Triangulated landmark points
    std::vector<int> match_indices_ref;                 ///< Feature indices in reference frame
    std::vector<int> match_indices_cur;                 ///< Feature indices in current frame
    std::string model_used;                             ///< "Homography" or "Fundamental"
};

/// @brief Monocular two-view initialization using H/F model scoring.
///
/// Implements the ORB-SLAM initialization strategy:
///   1. Extract features from two frames with sufficient parallax
///   2. Compute homography H and fundamental matrix F in parallel
///   3. Score models: RH = SH / (SH + SF). RH > 0.45 → planar (H),
///      else non-planar (E/F)
///   4. Decompose to recover [R|t] motion candidates
///   5. Triangulate all matches for each candidate
///   6. Select solution with most 3D points in front of both cameras
class MonocularInitializer {
public:
    struct Options {
        int min_features = 100;
        int min_matches = 100;
        int num_ransac_iterations = 200;
        double min_parallax_deg = 1.0;
        double max_reproj_error = 4.0;
        double hf_score_ratio = 0.45;
    };

    /// @param camera    Camera model for projection / unprojection.
    /// @param extractor ORB extractor for initialization (more features).
    /// @param opts      Configuration options.
    MonocularInitializer(const Camera& camera, features::OrbExtractor& extractor,
                         const Options& opts);

    /// @brief Process a frame for initialization.
    ///
    /// First call (no reference frame): stores frame as reference.
    /// Second call: matches features, computes H+F, recovers pose,
    /// triangulates, and returns the result.
    ///
    /// @param frame Current frame (features already extracted).
    /// @return Initialization result. success=true when map is created.
    InitializationResult Initialize(Frame& frame);

    /// @brief Reset to initial state.
    void Reset();

    /// @brief Whether a reference frame is stored.
    bool HasReference() const { return reference_frame_ != nullptr; }

private:
    const Camera& camera_;
    features::OrbExtractor& extractor_;
    Options opts_;

    std::shared_ptr<Frame> reference_frame_;

    /// Compute H and F with inlier masks using OpenCV.
    void ComputeModels(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
                       cv::Mat& H, cv::Mat& F, std::vector<uchar>& inliers_h,
                       std::vector<uchar>& inliers_f) const;

    /// Score H vs F. Returns true if H is the better model.
    bool SelectModel(const cv::Mat& H, const cv::Mat& F, const std::vector<cv::Point2f>& pts1,
                     const std::vector<cv::Point2f>& pts2, const std::vector<uchar>& inliers_h,
                     const std::vector<uchar>& inliers_f) const;

    /// Decompose H into [R|t] candidates (up to 8).
    std::vector<std::pair<Mat3, Vec3>> DecomposeH(const cv::Mat& H, const cv::Mat& K) const;

    /// Decompose E into [R|t] candidates (4).
    std::vector<std::pair<Mat3, Vec3>> DecomposeE(const cv::Mat& E, const cv::Mat& K,
                                                  const std::vector<cv::Point2f>& pts1_norm,
                                                  const std::vector<cv::Point2f>& pts2_norm) const;

    /// Test a motion candidate: triangulate and count valid points.
    int CheckMotion(const Mat3& R, const Vec3& t, const std::vector<cv::Point2f>& pts1,
                    const std::vector<cv::Point2f>& pts2, const cv::Mat& K,
                    std::vector<Vec3>& points_3d, std::vector<bool>& valid) const;
};

}  // namespace tracking
}  // namespace litevo
