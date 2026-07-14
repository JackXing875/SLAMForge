// =============================================================================
// Benchmark: Triangulation performance
// =============================================================================

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <benchmark/benchmark.h>

#include <random>

#include "litevo/core/camera.h"
#include "litevo/geometry/triangulation.h"

namespace {

/// Generate two camera poses and corresponding 2D observations.
struct TriangulationTestData {
    litevo::SE3 T1, T2;
    std::vector<cv::Point2f> pts1, pts2;
    std::vector<litevo::Vec3> ground_truth;
};

TriangulationTestData GenerateTestData(int num_points, const litevo::Camera& camera) {
    TriangulationTestData data;

    // Camera 1 at origin
    data.T1 = litevo::SE3::Identity();

    // Camera 2 moved 1m to the right
    data.T2 = litevo::SE3::Identity();
    data.T2.translation() = litevo::Vec3(1.0, 0.0, 0.0);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> xy_dist(-2.0, 2.0);
    std::uniform_real_distribution<double> z_dist(3.0, 15.0);
    std::normal_distribution<double> noise(0.0, 0.5);

    for (int i = 0; i < num_points; i++) {
        litevo::Vec3 p_w(xy_dist(rng), xy_dist(rng), z_dist(rng));
        data.ground_truth.push_back(p_w);

        litevo::Vec2 p1 = camera.Project(data.T1 * p_w);
        litevo::Vec2 p2 = camera.Project(data.T2 * p_w);

        data.pts1.emplace_back(static_cast<float>(p1.x() + noise(rng)),
                               static_cast<float>(p1.y() + noise(rng)));
        data.pts2.emplace_back(static_cast<float>(p2.x() + noise(rng)),
                               static_cast<float>(p2.y() + noise(rng)));
    }

    return data;
}

void BM_Triangulate_DLT(benchmark::State& state) {
    const int num_points = state.range(0);
    auto camera = MakeTestCamera();
    auto data = GenerateTestData(num_points, camera);

    cv::Mat K;
    cv::eigen2cv(camera.K(), K);

    cv::Mat P1, P2;
    {
        cv::Mat Rt1 = cv::Mat::eye(3, 4, CV_64F);
        P1 = K * Rt1;
    }
    {
        cv::Mat Rt2(3, 4, CV_64F);
        cv::Mat R2, t2;
        cv::eigen2cv(data.T2.rotation(), R2);
        cv::eigen2cv(data.T2.translation(), t2);
        R2.copyTo(Rt2(cv::Rect(0, 0, 3, 3)));
        t2.copyTo(Rt2(cv::Rect(3, 0, 1, 3)));
        P2 = K * Rt2;
    }

    for (auto _ : state) {
        cv::Mat pts4d;
        cv::triangulatePoints(P1, P2, data.pts1, data.pts2, pts4d);
        benchmark::DoNotOptimize(pts4d);
    }
    state.SetItemsProcessed(state.iterations() * num_points);
}
BENCHMARK(BM_Triangulate_DLT)->Arg(50)->Arg(100)->Arg(200)->Arg(500)->Unit(benchmark::kMicrosecond);

}  // namespace

BENCHMARK_MAIN();
