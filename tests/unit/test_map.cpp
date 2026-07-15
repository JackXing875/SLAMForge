// =============================================================================
// Map unit tests
// =============================================================================

#include <gtest/gtest.h>

#include "slamforge/core/frame.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"
#include "slamforge/features/orb_extractor.h"

using namespace slamforge;

class MapTest : public ::testing::Test {
protected:
    void SetUp() override {
        MapPoint::ResetIdCounter();
        Frame::ResetIdCounter();
    }
};

TEST_F(MapTest, AddMapPoint) {
    Map map;
    auto mp = map.AddMapPoint(Vec3(1, 2, 3), FrameId{0});
    EXPECT_NE(mp, nullptr);
    EXPECT_EQ(map.MapPointCount(), 1);
    EXPECT_NEAR(mp->Position().x(), 1.0, 1e-10);
}

TEST_F(MapTest, UniqueIds) {
    Map map;
    auto mp1 = map.AddMapPoint(Vec3(0, 0, 0), FrameId{0});
    auto mp2 = map.AddMapPoint(Vec3(1, 1, 1), FrameId{0});
    EXPECT_NE(mp1->Id().id, 0u);
    EXPECT_NE(mp2->Id().id, 0u);
    EXPECT_NE(mp1->Id(), mp2->Id());
}

TEST_F(MapTest, TwoViewLandmarkSurvivesInitialLifecycle) {
    Map map;
    auto mp = map.AddMapPoint(Vec3(0, 0, 5), FrameId{0});

    mp->AddObservation(FrameId{0});
    mp->AddObservation(FrameId{1});

    EXPECT_EQ(mp->Observations(), 2);
    EXPECT_FALSE(mp->IsBad());
    EXPECT_FALSE(mp->IsEraseReady(3));
    EXPECT_EQ(mp->FoundCount(), 2);
    EXPECT_FLOAT_EQ(mp->GetFoundRatio(), 1.0f);

    mp->SetFound(true);
    mp->ResetFound();
    EXPECT_FALSE(mp->IsFound());
    EXPECT_EQ(mp->FoundCount(), 2);

    // The grace period expires only after several mapping iterations.  A
    // triangulated point with two confirmed observations must still remain
    // usable rather than being treated as a bad point by default.
    mp->IncrementFrame();
    mp->IncrementFrame();
    mp->IncrementFrame();
    EXPECT_FALSE(mp->IsEraseReady(3));

    // Enough failed visibility predictions make the same low-observation
    // point eligible for culling.
    mp->IncreaseVisible(10);
    EXPECT_TRUE(mp->IsEraseReady(3));
}

TEST_F(MapTest, ResetKeepsMapPointZeroAsInvalidSentinel) {
    Map map;
    auto before_reset = map.AddMapPoint(Vec3(0, 0, 1), FrameId{0});
    EXPECT_NE(before_reset->Id().id, 0u);

    MapPoint::ResetIdCounter();
    auto after_reset = std::make_shared<MapPoint>(Vec3(1, 0, 1), FrameId{0});
    EXPECT_EQ(after_reset->Id().id, 1u);
}

TEST_F(MapTest, VisibilityIsCountedOncePerFrame) {
    auto mp = std::make_shared<MapPoint>(Vec3(0, 0, 5), FrameId{0});

    EXPECT_TRUE(mp->MarkObserved());
    mp->IncreaseVisible();
    EXPECT_FALSE(mp->MarkObserved());
    EXPECT_EQ(mp->VisibleCount(), 1);

    mp->ResetFound();
    EXPECT_TRUE(mp->MarkObserved());
    mp->IncreaseVisible();
    EXPECT_EQ(mp->VisibleCount(), 2);
}

TEST_F(MapTest, ReplacingMapPointExplicitlyInvalidatesAbsorbedPoint) {
    auto survivor = std::make_shared<MapPoint>(Vec3(0, 0, 5), FrameId{0});
    auto absorbed = std::make_shared<MapPoint>(Vec3(0.01, 0, 5), FrameId{1});
    survivor->AddObservation(FrameId{0});
    survivor->AddObservation(FrameId{1});
    absorbed->AddObservation(FrameId{2});

    survivor->Replace(absorbed.get());

    EXPECT_EQ(survivor->Observations(), 3);
    EXPECT_TRUE(absorbed->IsBad());
    EXPECT_TRUE(absorbed->IsEraseReady(3));
}

TEST_F(MapTest, GetMapPoint) {
    Map map;
    auto mp = map.AddMapPoint(Vec3(5, 5, 5), FrameId{0});
    auto found = map.GetMapPoint(mp->Id());
    EXPECT_NE(found, nullptr);
    EXPECT_NEAR(found->Position().z(), 5.0, 1e-10);

    // Non-existent ID
    auto missing = map.GetMapPoint(MapPointId{9999});
    EXPECT_EQ(missing, nullptr);
}

TEST_F(MapTest, EraseMapPoint) {
    Map map;
    auto mp = map.AddMapPoint(Vec3(0, 0, 10), FrameId{0});
    EXPECT_EQ(map.MapPointCount(), 1);

    map.EraseMapPoint(mp->Id());
    EXPECT_EQ(map.MapPointCount(), 0);
    EXPECT_EQ(map.GetMapPoint(mp->Id()), nullptr);
}

TEST_F(MapTest, GetAllMapPoints) {
    Map map;
    map.AddMapPoint(Vec3(0, 0, 0), FrameId{0});
    map.AddMapPoint(Vec3(1, 0, 0), FrameId{0});
    map.AddMapPoint(Vec3(2, 0, 0), FrameId{0});

    auto all = map.GetAllMapPoints();
    EXPECT_EQ(all.size(), 3);
}

TEST_F(MapTest, AddKeyFrame) {
    Map map;
    // Creating a real Frame requires an image + ORB extractor, so we test
    // with a nullptr for basic KeyFrame counting. In real usage, AddKeyFrame
    // takes shared_ptr<Frame>.
    // For now just verify the API compiles and functions:
    EXPECT_EQ(map.KeyFrameCount(), 0);
    EXPECT_EQ(map.ReferenceKeyFrame(), nullptr);
}

TEST_F(MapTest, Clear) {
    Map map;
    map.AddMapPoint(Vec3(0, 0, 0), FrameId{0});
    map.AddMapPoint(Vec3(1, 1, 1), FrameId{0});

    map.Clear();
    EXPECT_EQ(map.MapPointCount(), 0);
    EXPECT_EQ(map.KeyFrameCount(), 0);
}

TEST_F(MapTest, GetRecentKeyFrames) {
    Map map;
    // Empty map returns empty list
    auto recent = map.GetRecentKeyFrames(5);
    EXPECT_TRUE(recent.empty());
}
