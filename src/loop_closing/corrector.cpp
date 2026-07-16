// =============================================================================
// LoopCorrector implementation — temporally distributed Sim(3) correction
// =============================================================================

#include "slamforge/loop_closing/corrector.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"

namespace slamforge::loop_closing {

namespace {

double CorrectionAlpha(const LoopCorrection& correction, FrameId frame) {
    if (correction.current_frame.id <= correction.matched_frame.id ||
        frame.id <= correction.matched_frame.id) {
        return 0.0;
    }
    if (frame.id >= correction.current_frame.id) {
        return 1.0;
    }
    const uint64_t span = correction.current_frame.id - correction.matched_frame.id;
    // Keep a real neighborhood around the matched keyframe fixed. Starting a
    // large Sim(3) ramp at the exact matched frame deformed observations that
    // were already locally consistent with the anchor map. The remaining
    // loop path still has thousands of frames over which to absorb drift.
    const uint64_t fixed_prefix = std::min<uint64_t>(500, span / 4);
    const uint64_t correction_start = correction.matched_frame.id + fixed_prefix;
    if (frame.id <= correction_start) {
        return 0.0;
    }
    const double linear = static_cast<double>(frame.id - correction_start) /
                          static_cast<double>(correction.current_frame.id - correction_start);
    return linear * linear * (3.0 - 2.0 * linear);
}

}  // namespace

geometry::Sim3 InterpolateLoopCorrection(const LoopCorrection& correction, FrameId frame) {
    const double alpha = CorrectionAlpha(correction, frame);
    if (alpha <= 0.0) {
        return geometry::Sim3::Identity();
    }
    if (alpha >= 1.0) {
        return correction.transform;
    }

    geometry::Sim3 interpolated;
    const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond target(correction.transform.R);
    interpolated.R = identity.slerp(alpha, target).normalized().toRotationMatrix();
    interpolated.t = alpha * correction.transform.t;
    interpolated.s = std::pow(correction.transform.s, alpha);
    return interpolated;
}

SE3 ApplyLoopCorrectionToPose(const SE3& Tcw, const geometry::Sim3& correction) {
    if (!Tcw.matrix().allFinite() || !correction.R.allFinite() || !correction.t.allFinite() ||
        !std::isfinite(correction.s) || correction.s <= 0.0) {
        return Tcw;
    }

    const Vec3 old_center = -Tcw.rotation().transpose() * Tcw.translation();
    const Vec3 corrected_center = correction.TransformPoint(old_center);
    const Mat3 corrected_Rwc = correction.R * Tcw.rotation().transpose();

    SE3 corrected = SE3::Identity();
    corrected.linear() = corrected_Rwc.transpose();
    corrected.translation() = -corrected.rotation() * corrected_center;
    return corrected;
}

void LoopCorrector::CorrectLoop(std::shared_ptr<KeyFrame> current_kf,
                                std::shared_ptr<KeyFrame> matched_kf,
                                const geometry::Sim3& S_cw_correct,
                                const std::vector<std::pair<int, int>>& /*matches*/,
                                const std::vector<MapPointId>& matched_mps_cur,
                                const std::vector<MapPointId>& matched_mps_cand, Map& map) {
    if (!current_kf || !matched_kf || current_kf->Id().id <= matched_kf->Id().id ||
        !S_cw_correct.R.allFinite() || !S_cw_correct.t.allFinite() ||
        !std::isfinite(S_cw_correct.s) || S_cw_correct.s <= 0.0) {
        return;
    }

    auto graph_lock = map.AcquireGraphLock();
    const LoopCorrection correction{matched_kf->Id(), current_kf->Id(), S_cw_correct,
                                    static_cast<int>(matched_mps_cur.size())};

    // Distribute the closure over the complete temporal path.  This is the
    // built-in fallback for releases that do not ship g2o: the old region is
    // fixed, the current region receives the full Sim(3), and intermediate
    // keyframes receive a smooth fraction of the correction.
    const auto all_kfs = map.GetAllKeyFrames();
    for (const auto& kf : all_kfs) {
        if (!kf || kf->IsBad()) {
            continue;
        }
        const geometry::Sim3 local_correction = InterpolateLoopCorrection(correction, kf->Id());
        kf->SetPose(ApplyLoopCorrectionToPose(kf->Pose(), local_correction));
    }

    for (const auto& map_point : map.GetAllMapPoints()) {
        if (!map_point || map_point->IsBad()) {
            continue;
        }
        const geometry::Sim3 local_correction =
            InterpolateLoopCorrection(correction, map_point->ReferenceFrame());
        const Vec3 corrected_position = local_correction.TransformPoint(map_point->Position());
        if (corrected_position.allFinite()) {
            map_point->SetPosition(corrected_position);
        }
    }

    FuseMatchedMapPoints(matched_mps_cur, matched_mps_cand, map);

    // Only the two loop neighborhoods changed topology through fusion.  A
    // full O(K^2 * features) graph rebuild made long-video finalization
    // needlessly expensive.
    const auto updated_kfs = map.GetAllKeyFrames();
    current_kf->UpdateConnections(updated_kfs);
    matched_kf->UpdateConnections(updated_kfs);
}

void LoopCorrector::FuseMatchedMapPoints(const std::vector<MapPointId>& matched_mps_cur,
                                         const std::vector<MapPointId>& matched_mps_cand,
                                         Map& map) {
    const size_t count = std::min(matched_mps_cur.size(), matched_mps_cand.size());
    std::unordered_map<uint64_t, MapPointId> redirects;

    for (size_t index = 0; index < count; ++index) {
        const MapPointId current_id = matched_mps_cur[index];
        const MapPointId candidate_id = matched_mps_cand[index];
        if (current_id.id == 0 || candidate_id.id == 0 || current_id == candidate_id) {
            continue;
        }

        auto current = map.GetMapPoint(current_id);
        auto candidate = map.GetMapPoint(candidate_id);
        if (!current || !candidate || current->IsBad() || candidate->IsBad()) {
            continue;
        }

        std::shared_ptr<MapPoint> survivor;
        std::shared_ptr<MapPoint> absorbed;
        if (candidate->Observations() >= current->Observations()) {
            survivor = candidate;
            absorbed = current;
        } else {
            survivor = current;
            absorbed = candidate;
        }
        survivor->Replace(absorbed.get());
        redirects[absorbed->Id().id] = survivor->Id();
    }

    if (redirects.empty()) {
        return;
    }

    for (const auto& kf : map.GetAllKeyFrames()) {
        if (!kf) {
            continue;
        }
        for (int feature = 0; feature < kf->NumKeyPoints(); ++feature) {
            const MapPointId old_id = kf->MapPointIdAt(feature);
            const auto redirect = redirects.find(old_id.id);
            if (redirect != redirects.end()) {
                kf->SetMapPointId(feature, redirect->second);
            }
        }
    }
}

}  // namespace slamforge::loop_closing
