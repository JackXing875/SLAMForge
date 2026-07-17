// =============================================================================
// Dense depth calibration tests
// =============================================================================

#include <gtest/gtest.h>

#include <vector>

#include "slamforge/mapping/dense_mapper.h"

namespace slamforge::mapping {
namespace {

TEST(DenseDepthCalibrationTest, RecoversInverseDepthScaleAndShiftWithOutliers) {
    std::vector<DepthAnchor> anchors;
    for (int index = 0; index < 80; ++index) {
        const double depth = 0.8 + 0.045 * index;
        const double prediction = (1.0 / depth - 0.07) / 0.42;
        anchors.push_back({prediction, depth});
    }
    anchors.push_back({5.0, 0.2});
    anchors.push_back({0.1, 20.0});
    anchors.push_back({3.0, 12.0});

    const DepthCalibration calibration = CalibrateRelativeDepth(anchors, 20, 0.1);
    ASSERT_TRUE(calibration.valid());
    EXPECT_EQ(calibration.mode, DepthCalibration::Mode::kInverseDepth);
    EXPECT_NEAR(calibration.scale, 0.42, 0.02);
    EXPECT_NEAR(calibration.offset, 0.07, 0.02);
    EXPECT_LT(calibration.median_relative_error, 0.02);
    EXPECT_NEAR(calibration.Evaluate((1.0 / 3.25 - 0.07) / 0.42), 3.25, 0.08);
}

TEST(DenseDepthCalibrationTest, SupportsModelsThatPredictDirectDepth) {
    std::vector<DepthAnchor> anchors;
    for (int index = 0; index < 60; ++index) {
        const double prediction = 0.2 + 0.03 * index;
        anchors.push_back({prediction, 1.7 * prediction + 0.35});
    }

    const DepthCalibration calibration = CalibrateRelativeDepth(anchors, 20, 0.05);
    ASSERT_TRUE(calibration.valid());
    EXPECT_EQ(calibration.mode, DepthCalibration::Mode::kDirectDepth);
    EXPECT_NEAR(calibration.Evaluate(1.4), 2.73, 1e-6);
}

TEST(DenseDepthCalibrationTest, RejectsInsufficientOrInconsistentAnchors) {
    EXPECT_FALSE(CalibrateRelativeDepth({{1.0, 2.0}, {2.0, 3.0}}, 10, 0.2).valid());

    std::vector<DepthAnchor> inconsistent;
    for (int index = 0; index < 30; ++index) {
        inconsistent.push_back({static_cast<double>(index % 3),
                                0.5 + static_cast<double>((index * 17) % 29)});
    }
    EXPECT_FALSE(CalibrateRelativeDepth(inconsistent, 20, 0.05).valid());
}

}  // namespace
}  // namespace slamforge::mapping
