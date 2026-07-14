// =============================================================================
// PoseGraphOptimizer implementation — g2o pose graph optimization
// =============================================================================

#include "litevo/loop_closing/pose_graph.h"

#ifdef LITEVO_HAS_G2O

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <unordered_map>

#include "litevo/core/keyframe.h"
#include "litevo/core/map.h"
#include "litevo/geometry/se3.h"

namespace litevo::loop_closing {

PoseGraphOptimizer::PoseGraphOptimizer(const PoseGraphConfig& config) : config_(config) {}

PoseGraphOptimizer::~PoseGraphOptimizer() = default;

void PoseGraphOptimizer::Optimize(Map& map, std::shared_ptr<KeyFrame> loop_kf,
                                  std::shared_ptr<KeyFrame> loop_kf_matched,
                                  const SE3& relative_pose, const Mat6& information) {
    if (!loop_kf || !loop_kf_matched)
        return;

    // Setup g2o optimizer
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
    auto linear_solver =
        std::make_unique<g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>>();
    auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
    auto* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(config_.verbose);

    // ── Add vertices: one per keyframe ────────────────────────────────────

    auto all_kfs = map.GetAllKeyFrames();
    std::unordered_map<FrameId, g2o::VertexSE3*, FrameIdHash> vertex_map;

    // Fix the first keyframe (gauge freedom)
    bool first = true;

    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;

        auto* vertex = new g2o::VertexSE3();
        vertex->setId(static_cast<int>(kf->Id().id));

        const SE3& Tcw = kf->Pose();
        Eigen::Quaterniond quat(Tcw.rotation());
        Eigen::Vector3d trans = Tcw.translation();

        g2o::SE3Quat pose(quat, trans);
        vertex->setEstimate(pose);

        if (first) {
            vertex->setFixed(true);
            first = false;
        }
        // Fix loop_kf_matched to avoid gauge ambiguity during correction
        if (kf.get() == loop_kf_matched.get()) {
            // Don't fix the matched KF — it should be adjustable
            // Only the very first KF is fixed
        }

        optimizer.addVertex(vertex);
        vertex_map[kf->Id()] = vertex;
    }

    // ── Add spanning tree edges ────────────────────────────────────────────

    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;

        KeyFrame* parent = kf->GetParent();
        if (!parent || parent->IsBad())
            continue;

        auto vit1 = vertex_map.find(kf->Id());
        auto vit2 = vertex_map.find(parent->Id());
        if (vit1 == vertex_map.end() || vit2 == vertex_map.end())
            continue;

        // Relative transform: T_parent_cw * T_kf_wc
        SE3 T_rel = geometry::RelativePose(parent->Pose(), kf->Pose());
        Eigen::Quaterniond q_rel(T_rel.rotation());
        Eigen::Vector3d t_rel = T_rel.translation();

        auto* edge = new g2o::EdgeSE3();
        edge->setVertex(0, vit2->second);  // parent
        edge->setVertex(1, vit1->second);  // child
        edge->setMeasurement(g2o::SE3Quat(q_rel, t_rel));

        // Information: identity (spanning tree edge is soft constraint)
        Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity();
        edge->setInformation(info);

        auto* robust = new g2o::RobustKernelHuber();
        robust->setDelta(1.0);
        edge->setRobustKernel(robust);

        optimizer.addEdge(edge);
    }

    // ── Add covisibility edges (weight ≥ threshold) ───────────────────────

    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;

        auto covisibles = kf->GetCovisiblesByWeight(config_.min_covisibility_weight);
        for (auto* cov_kf : covisibles) {
            if (!cov_kf || cov_kf->IsBad())
                continue;
            if (cov_kf->Id().id <= kf->Id().id)
                continue;  // Avoid duplicate edges

            auto vit1 = vertex_map.find(kf->Id());
            auto vit2 = vertex_map.find(cov_kf->Id());
            if (vit1 == vertex_map.end() || vit2 == vertex_map.end())
                continue;

            SE3 T_rel = geometry::RelativePose(kf->Pose(), cov_kf->Pose());
            Eigen::Quaterniond q_rel(T_rel.rotation());
            Eigen::Vector3d t_rel = T_rel.translation();

            auto* edge = new g2o::EdgeSE3();
            edge->setVertex(0, vit1->second);
            edge->setVertex(1, vit2->second);
            edge->setMeasurement(g2o::SE3Quat(q_rel, t_rel));

            Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity();
            edge->setInformation(info);

            auto* robust = new g2o::RobustKernelHuber();
            robust->setDelta(1.0);
            edge->setRobustKernel(robust);

            optimizer.addEdge(edge);
        }
    }

    // ── Add loop closure edge ──────────────────────────────────────────────

    {
        auto vit1 = vertex_map.find(loop_kf_matched->Id());
        auto vit2 = vertex_map.find(loop_kf->Id());
        if (vit1 != vertex_map.end() && vit2 != vertex_map.end()) {
            Eigen::Quaterniond q_rel(relative_pose.rotation());
            Eigen::Vector3d t_rel = relative_pose.translation();

            auto* edge = new g2o::EdgeSE3();
            edge->setVertex(0, vit1->second);  // matched KF
            edge->setVertex(1, vit2->second);  // current (loop) KF
            edge->setMeasurement(g2o::SE3Quat(q_rel, t_rel));

            // Information matrix
            Eigen::Matrix<double, 6, 6> info =
                Eigen::Matrix<double, 6, 6>::Identity() * config_.loop_edge_weight;
            edge->setInformation(info);

            auto* robust = new g2o::RobustKernelHuber();
            robust->setDelta(1.0);
            edge->setRobustKernel(robust);

            optimizer.addEdge(edge);
        }
    }

    // ── Optimize ───────────────────────────────────────────────────────────

    optimizer.initializeOptimization();
    optimizer.optimize(config_.max_iterations);

    // ── Update keyframe poses ──────────────────────────────────────────────

    for (auto& kf : all_kfs) {
        if (!kf || kf->IsBad())
            continue;

        auto it = vertex_map.find(kf->Id());
        if (it == vertex_map.end())
            continue;

        g2o::VertexSE3* vertex = it->second;
        kf->SetPose(SE3(vertex->estimate().matrix()));
    }
}

}  // namespace litevo::loop_closing

#else  // !LITEVO_HAS_G2O

#include "litevo/core/keyframe.h"
#include "litevo/core/map.h"

namespace litevo::loop_closing {

PoseGraphOptimizer::PoseGraphOptimizer(const PoseGraphConfig& config) : config_(config) {}

PoseGraphOptimizer::~PoseGraphOptimizer() = default;

void PoseGraphOptimizer::Optimize(Map&, std::shared_ptr<KeyFrame>, std::shared_ptr<KeyFrame>,
                                  const SE3&, const Mat6&) {
    // No-op when g2o is not available
}

}  // namespace litevo::loop_closing

#endif  // LITEVO_HAS_G2O
