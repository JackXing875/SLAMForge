// =============================================================================
// Configuration loading unit tests
// =============================================================================

#include <gtest/gtest.h>

#include "litevo/core/config.h"

using namespace litevo;

TEST(ConfigTest, DefaultConfig) {
    SystemConfig cfg = SystemConfig::Default();
    EXPECT_DOUBLE_EQ(cfg.camera.fx, 718.856);
    EXPECT_DOUBLE_EQ(cfg.camera.fy, 718.856);
    EXPECT_EQ(cfg.orb.num_features, 1200);
    EXPECT_EQ(cfg.tracking.min_features_for_tracking, 50);
    EXPECT_EQ(cfg.mapping.sliding_window_size, 20);
    EXPECT_FALSE(cfg.loop_closing.enabled);
    EXPECT_EQ(cfg.output_format, "tum");
}

#ifdef LITEVO_HAS_YAML_CPP
TEST(ConfigTest, LoadFromYAML) {
    // This test requires a YAML file on disk
    // Create a temporary config
    auto cfg_opt = SystemConfig::LoadFromYAML("config/default.yaml");
    EXPECT_TRUE(cfg_opt.has_value());
    if (cfg_opt) {
        EXPECT_GT(cfg_opt->camera.width, 0);
        EXPECT_GT(cfg_opt->camera.height, 0);
    }
}

TEST(ConfigTest, LoadInvalidFile) {
    auto cfg_opt = SystemConfig::LoadFromYAML("nonexistent_file.yaml");
    // Should return nullopt on parse error, or return Default() if YAML not available
    EXPECT_FALSE(cfg_opt.has_value());
}
#endif

TEST(ConfigTest, CameraConfigDefaults) {
    CameraConfig cam;
    EXPECT_EQ(cam.width, 1241);  // KITTI default
    EXPECT_EQ(cam.height, 376);
    EXPECT_DOUBLE_EQ(cam.k1, 0.0);
    EXPECT_DOUBLE_EQ(cam.k2, 0.0);
}

TEST(ConfigTest, TrackingConfigDefaults) {
    TrackingConfig trk;
    EXPECT_EQ(trk.min_features_for_tracking, 50);
    EXPECT_EQ(trk.max_frames_between_kf, 40);
    EXPECT_GT(trk.max_reprojection_error, 0);
}
