// =============================================================================
// LiteVO camera model — pinhole with radial-tangential distortion
// =============================================================================

#pragma once

#include <Eigen/Core>

#include "litevo/core/types.h"

namespace litevo {

/// @brief Pinhole camera model with radial-tangential distortion.
///
/// Supports:
///   - Projection:  3D world point -> 2D pixel (with distortion)
///   - Unprojection: 2D pixel -> 3D ray (undistorted)
///   - Distortion/undistortion of image points
class Camera {
public:
    Camera() = default;

    /// @brief Construct from intrinsic parameters.
    /// @param fx, fy Focal lengths in pixels
    /// @param cx, cy Principal point in pixels
    /// @param width, height Image dimensions
    Camera(double fx, double fy, double cx, double cy, int width, int height);

    /// @brief Set radial-tangential distortion coefficients.
    void SetDistortion(double k1, double k2, double p1, double p2, double k3 = 0.0);

    // ── Projection ────────────────────────────────────────────────────────

    /// @brief Project a 3D camera-frame point to pixel coordinates (with distortion).
    /// @param p_cam 3D point in camera frame
    /// @return 2D pixel coordinates, or (-1,-1) if behind camera
    [[nodiscard]] Vec2 Project(const Vec3& p_cam) const;

    /// @brief Project a 3D world point using the camera pose.
    /// @param p_w 3D point in world frame
    /// @param T_cw World-to-camera transformation
    /// @return 2D pixel coordinates
    [[nodiscard]] Vec2 ProjectWorld(const Vec3& p_w, const SE3& T_cw) const;

    // ── Unprojection ──────────────────────────────────────────────────────

    /// @brief Unproject a pixel to a 3D ray in camera frame (undistorted).
    /// @param pixel Pixel coordinates (u, v)
    /// @return Unit-norm ray direction in camera frame
    [[nodiscard]] Vec3 Unproject(const Vec2& pixel) const;

    // ── Distortion ────────────────────────────────────────────────────────

    /// @brief Apply radial-tangential distortion to normalized coordinates.
    [[nodiscard]] Vec2 Distort(const Vec2& normalized) const;

    /// @brief Remove distortion from normalized coordinates (iterative).
    [[nodiscard]] Vec2 Undistort(const Vec2& normalized) const;

    // ── Accessors ─────────────────────────────────────────────────────────

    [[nodiscard]] const Mat3& K()          const { return K_; }
    [[nodiscard]] const Mat3& K_inv()      const { return K_inv_; }
    [[nodiscard]] double      fx()         const { return fx_; }
    [[nodiscard]] double      fy()         const { return fy_; }
    [[nodiscard]] double      cx()         const { return cx_; }
    [[nodiscard]] double      cy()         const { return cy_; }
    [[nodiscard]] int         width()      const { return width_; }
    [[nodiscard]] int         height()     const { return height_; }
    [[nodiscard]] bool        has_distortion() const { return has_distortion_; }

    /// @brief Check if a pixel is within image bounds.
    [[nodiscard]] bool IsInImage(const Vec2& pixel, int border = 0) const;

    /// @brief Compute the projection matrix P = K * [R | t].
    [[nodiscard]] Mat34 ProjectionMatrix(const SE3& T_cw) const;

    /// @brief Create from a SystemConfig (Phase 2e will implement config parsing).
    struct CameraParams {
        double fx, fy, cx, cy;
        double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
        int width = 640, height = 480;
    };
    static Camera FromParams(const CameraParams& params);

private:
    Mat3 K_{Mat3::Identity()};
    Mat3 K_inv_{Mat3::Identity()};

    double fx_{0}, fy_{0}, cx_{0}, cy_{0};
    int    width_{0}, height_{0};

    // Distortion: radial-tangential model
    double k1_{0}, k2_{0}, p1_{0}, p2_{0}, k3_{0};
    bool   has_distortion_{false};

    static constexpr int kMaxDistortionIterations = 10;
    static constexpr double kDistortionEpsilon = 1e-8;
};

}  // namespace litevo
