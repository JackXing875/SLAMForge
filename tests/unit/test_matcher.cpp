// =============================================================================
// FeatureMatcher regression tests
// =============================================================================

#include <gtest/gtest.h>

#include "slamforge/tracking/matcher.h"

using slamforge::tracking::FeatureMatcher;

TEST(FeatureMatcherTest, RotationHistogramWrapsDegreeAnglesAtFullTurn) {
    std::vector<cv::KeyPoint> reference;
    std::vector<cv::KeyPoint> current;
    std::vector<std::pair<int, int>> matches;

    // The boundary value 359° must share the same cyclic bin as 0° instead
    // of indexing beyond the fixed 30-bin rotation histogram.
    for (int index = 0; index < 12; ++index) {
        cv::KeyPoint ref(cv::Point2f(static_cast<float>(index), 0.0f), 1.0f);
        ref.angle = index < 6 ? 359.0f : 0.0f;
        cv::KeyPoint cur(cv::Point2f(static_cast<float>(index), 0.0f), 1.0f);
        cur.angle = 0.0f;
        reference.push_back(ref);
        current.push_back(cur);
        matches.emplace_back(index, index);
    }

    const auto filtered = FeatureMatcher::FilterByRotationHistogram(matches, reference, current);

    EXPECT_EQ(filtered.size(), matches.size());
}
