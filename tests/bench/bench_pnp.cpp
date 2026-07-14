// =============================================================================
// Benchmark: PnP pose estimation performance
// =============================================================================

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <benchmark/benchmark.h>

#include <random>

#include "litevo/core/camera.h"
#include "litevo/geometry/pnp.h"

namespace {

litevo::Camera MakeTestCamera() {
    return litevo::Camera(718.856, 718.856, 607.193, 185.216, 1241, 376);
}

/// Generate random 3D points and their 2D projections with noise.
struct TestData {
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    litevo::SE3 Tcw_gt;
};

TestData GenerateTestData(int num_points, const litevo::Camera& camera) {
    TestData data;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> pos_dist(-5.0, 5.0);
    std::uniform_real_distribution<double> z_dist(1.0, 20.0);
    std::normal_distribution<double> noise(0.0, 0.5);  // 0.5 pixel noise

    // Ground truth pose: looking forward
    data.Tcw_gt = litevo::SE3::Identity();
    data.Tcw_gt.translation() = litevo::Vec3(0, 0, 0);

    for (int i = 0; i < num_points; i++) {
        litevo::Vec3 p_w(pos_dist(rng), pos_dist(rng), z_dist(rng));
        litevo::Vec3 p_c = data.Tcw_gt * p_w;
        litevo::Vec2 pixel = camera.Project(p_c);

        cv::Point3f pt3(static_cast<float>(p_w.x()), static_cast<float>(p_w.y()),
                        static_cast<float>(p_w.z()));
        cv::Point2f pt2(static_cast<float>(pixel.x() + noise(rng)),
                        static_cast<float>(pixel.y() + noise(rng)));

        data.pts_3d.push_back(pt3);
        data.pts_2d.push_back(pt2);
    }

    return data;
}

void BM_PnP_RANSAC(benchmark::State& state) {
    const int num_points = state.range(0);
    auto camera = MakeTestCamera();
    auto data = GenerateTestData(num_points, camera);

    cv::Mat K;
    cv::eigen2cv(camera.K(), K);

    for (auto _ : state) {
        cv::Mat rvec, tvec;
        cv::Mat inliers;
        bool ok = cv::solvePnPRansac(data.pts_3d, data.pts_2d, K, cv::Mat(), rvec, tvec, false, 100,
                                     4.0, 0.99, inliers);
        benchmark::DoNotOptimize(ok);
    }
    state.SetItemsProcessed(state.iterations() * num_points);
}
BENCHMARK(BM_PnP_RANSAC)->Arg(50)->Arg(100)->Arg(200)->Arg(500)->Unit(benchmark::kMicrosecond);

void BM_PnP_EPnP(benchmark::State& state) {
    const int num_points = state.range(0);
    auto camera = MakeTestCamera();
    auto data = GenerateTestData(num_points, camera);

    cv::Mat K;
    cv::eigen2cv(camera.K(), K);

    for (auto _ : state) {
        cv::Mat rvec, tvec;
        bool ok = cv::solvePnP(data.pts_3d, data.pts_2d, K, cv::Mat(), rvec, tvec, false,
                               cv::SOLVEPNP_EPNP);
        benchmark::DoNotOptimize(ok);
    }
    state.SetItemsProcessed(state.iterations() * num_points);
}
BENCHMARK(BM_PnP_EPnP)->Arg(50)->Arg(100)->Arg(200)->Arg(500)->Unit(benchmark::kMicrosecond);

}  // namespace

BENCHMARK_MAIN();
