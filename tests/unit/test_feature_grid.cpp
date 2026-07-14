// =============================================================================
// FeatureGrid unit tests
// =============================================================================

#include <gtest/gtest.h>

#include "litevo/core/feature_grid.h"

using namespace litevo;

TEST(FeatureGridTest, Construction) {
    FeatureGrid grid(640, 480, 64);
    EXPECT_EQ(grid.GridCols(), 10);  // ceil(640/64) = 10
    EXPECT_EQ(grid.GridRows(), 8);   // ceil(480/64) = 7.5 → 8
    EXPECT_EQ(grid.CellSize(), 64);
}

TEST(FeatureGridTest, SingleFeatureRetrieval) {
    FeatureGrid grid(640, 480, 64);

    std::vector<cv::KeyPoint> kps;
    kps.emplace_back(cv::Point2f(100.0f, 200.0f), 1.0f);  // index 0

    grid.Assign(kps);

    auto candidates = grid.GetCandidates(100.0f, 200.0f, 10.0f);
    EXPECT_EQ(candidates.size(), 1);
    EXPECT_EQ(candidates[0], 0);
}

TEST(FeatureGridTest, SearchWindow) {
    FeatureGrid grid(640, 480, 64);

    std::vector<cv::KeyPoint> kps;
    kps.emplace_back(cv::Point2f(50.0f, 50.0f), 1.0f);    // index 0
    kps.emplace_back(cv::Point2f(200.0f, 50.0f), 1.0f);   // index 1
    kps.emplace_back(cv::Point2f(50.0f, 300.0f), 1.0f);   // index 2
    kps.emplace_back(cv::Point2f(200.0f, 300.0f), 1.0f);  // index 3

    grid.Assign(kps);

    // Search near (200, 50) — should only find index 1
    auto candidates = grid.GetCandidates(200.0f, 50.0f, 30.0f);
    EXPECT_GE(candidates.size(), 1);

    // Index 1 should be in the results
    bool found_1 = false;
    for (int idx : candidates) {
        if (idx == 1)
            found_1 = true;
    }
    EXPECT_TRUE(found_1);

    // Search near (50, 300) — should only find index 2
    auto candidates2 = grid.GetCandidates(50.0f, 300.0f, 30.0f);
    EXPECT_GE(candidates2.size(), 1);

    bool found_2 = false;
    for (int idx : candidates2) {
        if (idx == 2)
            found_2 = true;
    }
    EXPECT_TRUE(found_2);
}

TEST(FeatureGridTest, BoundaryClamping) {
    FeatureGrid grid(640, 480, 64);

    std::vector<cv::KeyPoint> kps;
    kps.emplace_back(cv::Point2f(10.0f, 10.0f), 1.0f);  // index 0, top-left corner

    grid.Assign(kps);

    // Search outside the image — should not crash
    auto candidates = grid.GetCandidates(-100.0f, -100.0f, 50.0f);
    // May or may not find index 0 depending on radius extending into image
    EXPECT_NO_THROW(grid.GetCandidates(-9999.0f, -9999.0f, 5.0f));

    // Search far outside — should return empty
    auto empty = grid.GetCandidates(99999.0f, 99999.0f, 5.0f);
    EXPECT_TRUE(empty.empty());
}

TEST(FeatureGridTest, EmptyGrid) {
    FeatureGrid grid(640, 480, 64);

    // Grid with no features assigned should return empty candidates
    auto candidates = grid.GetCandidates(320.0f, 240.0f, 100.0f);
    EXPECT_TRUE(candidates.empty());
}

TEST(FeatureGridTest, MultipleFeaturesSameCell) {
    FeatureGrid grid(640, 480, 64);

    std::vector<cv::KeyPoint> kps;
    // All in cell (0, 0) since 30 < 64
    kps.emplace_back(cv::Point2f(10.0f, 10.0f), 1.0f);  // 0
    kps.emplace_back(cv::Point2f(20.0f, 20.0f), 1.0f);  // 1
    kps.emplace_back(cv::Point2f(30.0f, 30.0f), 1.0f);  // 2

    grid.Assign(kps);

    auto candidates = grid.GetCandidates(20.0f, 20.0f, 50.0f);
    EXPECT_EQ(candidates.size(), 3);
}
