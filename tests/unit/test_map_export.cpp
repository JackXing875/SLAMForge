#include <Eigen/Core>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"
#include "slamforge/io/map_export.h"

namespace {

TEST(MapExport, WritesOnlyValidMapPoints) {
    slamforge::Map map;
    map.AddMapPoint(slamforge::Vec3(1.25, -2.5, 3.75), slamforge::FrameId{1});
    auto bad_point = map.AddMapPoint(slamforge::Vec3(4.0, 5.0, 6.0), slamforge::FrameId{1});
    bad_point->SetBad();
    map.AddMapPoint(slamforge::Vec3(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
                    slamforge::FrameId{1});

    const std::filesystem::path output =
        std::filesystem::path(::testing::TempDir()) / "slamforge-map-export.ply";
    const auto result = slamforge::io::ExportMapAsPly(map, output);

    ASSERT_TRUE(result) << result.error;
    EXPECT_EQ(result.point_count, 1U);

    std::ifstream input(output);
    ASSERT_TRUE(input.is_open());
    std::ostringstream buffer;
    buffer << input.rdbuf();
    const std::string contents = buffer.str();
    EXPECT_NE(contents.find("element vertex 1"), std::string::npos);
    EXPECT_NE(contents.find("1.25 -2.5 3.75"), std::string::npos);

    std::error_code error;
    std::filesystem::remove(output, error);
}

TEST(MapExport, ReportsUnwritableOutput) {
    slamforge::Map map;
    const std::filesystem::path output =
        std::filesystem::path(::testing::TempDir()) / "missing-directory" / "map.ply";

    const auto result = slamforge::io::ExportMapAsPly(map, output);

    EXPECT_FALSE(result);
    EXPECT_FALSE(result.error.empty());
}

}  // namespace
