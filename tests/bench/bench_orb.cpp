// =============================================================================
// Benchmark: ORB feature extraction throughput
// =============================================================================

#include <benchmark/benchmark.h>
#include <opencv2/imgproc.hpp>

#include "litevo/features/orb_extractor.h"

namespace {

/// Generate a synthetic grayscale image of given size with noise.
cv::Mat MakeSyntheticImage(int width, int height) {
    cv::Mat img(height, width, CV_8UC1);
    cv::randu(img, 0, 255);
    return img;
}

void BM_OrbExtract_KITTI(benchmark::State& state) {
    litevo::features::OrbExtractor::Options opts;
    opts.num_features = 1200;
    opts.scale_factor = 1.2;
    opts.num_levels = 8;
    opts.ini_threshold = 20;
    opts.min_threshold = 7;
    litevo::features::OrbExtractor extractor(opts);

    cv::Mat img = MakeSyntheticImage(1241, 376);
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;

    for (auto _ : state) {
        kps.clear();
        desc.release();
        int n = extractor.ExtractUniform(img, kps, desc);
        benchmark::DoNotOptimize(n);
    }
    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_OrbExtract_KITTI)->Unit(benchmark::kMillisecond);

void BM_OrbExtract_VGA(benchmark::State& state) {
    litevo::features::OrbExtractor::Options opts;
    opts.num_features = 1000;
    opts.scale_factor = 1.2;
    opts.num_levels = 8;
    litevo::features::OrbExtractor extractor(opts);

    cv::Mat img = MakeSyntheticImage(640, 480);
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;

    for (auto _ : state) {
        kps.clear();
        desc.release();
        int n = extractor.ExtractUniform(img, kps, desc);
        benchmark::DoNotOptimize(n);
    }
    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_OrbExtract_VGA)->Unit(benchmark::kMillisecond);

void BM_OrbExtract_HD(benchmark::State& state) {
    litevo::features::OrbExtractor::Options opts;
    opts.num_features = 2000;
    opts.scale_factor = 1.2;
    opts.num_levels = 8;
    litevo::features::OrbExtractor extractor(opts);

    cv::Mat img = MakeSyntheticImage(1920, 1080);
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;

    for (auto _ : state) {
        kps.clear();
        desc.release();
        int n = extractor.ExtractUniform(img, kps, desc);
        benchmark::DoNotOptimize(n);
    }
    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_OrbExtract_HD)->Unit(benchmark::kMillisecond);

}  // namespace

BENCHMARK_MAIN();
