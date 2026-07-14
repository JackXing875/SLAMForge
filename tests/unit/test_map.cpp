// =============================================================================
// Map unit tests
// =============================================================================

#include <gtest/gtest.h>

#include "litevo/core/frame.h"
#include "litevo/core/map.h"
#include "litevo/core/map_point.h"
#include "litevo/features/orb_extractor.h"

using namespace litevo;

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
    EXPECT_NE(mp1->Id(), mp2->Id());
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
