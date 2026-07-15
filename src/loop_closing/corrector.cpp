// =============================================================================
// LoopCorrector implementation — Sim(3) propagation and map point fusion
// =============================================================================

#include "litevo/loop_closing/corrector.h"

#include <algorithm>
#include <unordered_set>

#include "litevo/core/camera.h"
#include "litevo/core/keyframe.h"
#include "litevo/core/map.h"
#include "litevo/core/map_point.h"
#include "litevo/geometry/se3.h"

namespace litevo::loop_closing {

void LoopCorrector::CorrectLoop(std::shared_ptr<KeyFrame> current_kf,
                                std::shared_ptr<KeyFrame> matched_kf,
                                const geometry::Sim3& S_cw_correct,
                                const std::vector<std::pair<int, int>>& /*matches*/,
                                const std::vector<MapPointId>& matched_mps_cur,
                                const std::vector<MapPointId>& matched_mps_cand, Map& map) {
    if (!current_kf || !matched_kf)
        return;

    // CorrectLoop is also a public component API.  Take the graph guard here
    // in addition to LoopClosing's outer transaction so direct callers cannot
    // race the tracking/local-mapping workers either.
    auto graph_lock = map.AcquireGraphLock();

    // Step 1: Update current KF pose with Sim(3) correction.
    // Capture every relative transform from the uncorrected graph first.  If
    // current_kf is overwritten before those transforms are computed, the
    // correction is implicitly applied a second time while propagating it to
    // its neighbours.
    const SE3 Tcw_current_original = current_kf->Pose();
    const SE3 Tcw_matched_original = matched_kf->Pose();
    std::vector<std::pair<KeyFrame*, SE3>> covisible_original_poses;
    const auto covisibles = current_kf->GetBestCovisibilityKeyFrames(20);
    covisible_original_poses.reserve(covisibles.size());
    for (auto* cov_kf : covisibles) {
        if (!cov_kf || cov_kf->IsBad() || cov_kf == current_kf.get()) {
            continue;
        }
        covisible_original_poses.emplace_back(cov_kf, cov_kf->Pose());
    }

    // The correction maps: current KF pose → corrected pose
    // T_cw_corrected = S_cw_correct * T_cw_original
    SE3 Tcw_corrected;
    Tcw_corrected.linear() = S_cw_correct.R * Tcw_current_original.rotation();
    Tcw_corrected.translation() =
        S_cw_correct.s * (S_cw_correct.R * Tcw_current_original.translation()) +
        S_cw_correct.t;
    current_kf->SetPose(Tcw_corrected);

    // Step 2: Propagate correction through covisibility graph
    std::set<KeyFrame*> corrected_kfs;
    corrected_kfs.insert(current_kf.get());

    // Propagate to covisible neighbors while preserving their original
    // relative transform to the current keyframe.
    for (const auto& [cov_kf, Tcw_cov_original] : covisible_original_poses) {
        if (corrected_kfs.contains(cov_kf)) {
            continue;
        }

        const SE3 T_rel =
            geometry::RelativePose(Tcw_current_original, Tcw_cov_original);

        // Apply correction: T'cov = Tcw_corrected * T_rel
        SE3 Tcov_new;
        Tcov_new.linear() = Tcw_corrected.rotation() * T_rel.rotation();
        Tcov_new.translation() =
            Tcw_corrected.rotation() * T_rel.translation() + Tcw_corrected.translation();
        cov_kf->SetPose(Tcov_new);
        corrected_kfs.insert(cov_kf);
    }

    // Also apply to the matched-KF side using its original relative pose.
    if (matched_kf.get() != current_kf.get()) {
        matched_kf->SetPose(Tcw_corrected *
                            geometry::RelativePose(Tcw_current_original, Tcw_matched_original));
    }

    // Step 3: Fuse duplicated map points
    // Build list of keyframes involved in the loop
    std::vector<std::shared_ptr<KeyFrame>> loop_kfs;
    auto all_kfs = map.GetAllKeyFrames();
    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;
        if (corrected_kfs.contains(kf.get())) {
            loop_kfs.push_back(kf);
        }
    }
    // Add matched KF side covisibles
    auto matched_covis = matched_kf->GetBestCovisibilityKeyFrames(20);
    for (auto* cov_kf : matched_covis) {
        if (!cov_kf || cov_kf->IsBad())
            continue;
        bool found = false;
        for (const auto& kf : loop_kfs) {
            if (kf.get() == cov_kf) {
                found = true;
                break;
            }
        }
        if (!found) {
            // Find the shared_ptr
            for (auto& akf : all_kfs) {
                if (akf.get() == cov_kf) {
                    loop_kfs.push_back(akf);
                    break;
                }
            }
        }
    }

    // Fuse on matched side
    SearchAndFuse(loop_kfs, matched_mps_cand, 4.0, map);

    // Fuse on current side
    SearchAndFuse(loop_kfs, matched_mps_cur, 4.0, map);

    // Step 4: Update covisibility connections
    UpdateConnections(map);
}

void LoopCorrector::SearchAndFuse(const std::vector<std::shared_ptr<KeyFrame>>& /*loop_kfs*/,
                                  const std::vector<MapPointId>& matched_mps,
                                  double /*max_reproj_error*/, Map& map) {
    // For each matched map point, search for nearby map points in loop KFs
    // that might represent the same 3D point and fuse them.

    const auto all_kfs = map.GetAllKeyFrames();
    const auto all_mps = map.GetAllMapPoints();
    for (const MapPointId& mp_id : matched_mps) {
        auto mp = map.GetMapPoint(mp_id);
        if (!mp || mp->IsBad())
            continue;

        Vec3 pos = mp->Position();

        for (auto& other_mp : all_mps) {
            if (!other_mp || other_mp->IsBad())
                continue;
            if (other_mp->Id() == mp->Id())
                continue;

            // Check 3D distance
            double dist = (other_mp->Position() - pos).norm();
            if (dist > 0.1)
                continue;  // 10cm threshold for fusion

            // Check descriptor similarity
            if (mp->Descriptor().rows > 0 && other_mp->Descriptor().rows > 0) {
                auto dist_hamming =
                    cv::norm(mp->Descriptor(), other_mp->Descriptor(), cv::NORM_HAMMING);
                if (dist_hamming > 100)
                    continue;  // Hamming distance too high
            }

            // Fuse: keep the one with more observations.  Replace() marks
            // its argument bad, so associations must always be redirected
            // from the absorbed landmark to the surviving one.
            std::shared_ptr<MapPoint> survivor;
            std::shared_ptr<MapPoint> absorbed;
            if (mp->Observations() >= other_mp->Observations()) {
                survivor = mp;
                absorbed = other_mp;
            } else {
                survivor = other_mp;
                absorbed = mp;
            }
            survivor->Replace(absorbed.get());

            // Associations outside the immediate loop neighbourhood can also
            // reference the absorbed point.  Rewrite the entire map so later
            // tracking never follows a MapPoint that has just been marked bad.
            for (const auto& kf : all_kfs) {
                if (!kf)
                    continue;
                for (int i = 0; i < kf->NumKeyPoints(); ++i) {
                    MapPointId kf_mp_id = kf->MapPointIdAt(i);
                    if (kf_mp_id == absorbed->Id()) {
                        kf->SetMapPointId(i, survivor->Id());
                    }
                }
            }

            // Continue fusing around the surviving landmark if the original
            // query point was absorbed by a better-observed candidate.
            mp = survivor;
            pos = mp->Position();
        }
    }
}

void LoopCorrector::UpdateConnections(Map& map) {
    auto all_kfs = map.GetAllKeyFrames();

    // Update covisibility for all keyframes
    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;
        kf->UpdateConnections(all_kfs);
    }
}

}  // namespace litevo::loop_closing
