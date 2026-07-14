// =============================================================================
// LiteVO camera model — implementation
// =============================================================================

#include "litevo/core/camera.h"

#include <cmath>

namespace litevo {

Camera::Camera(double fx, double fy, double cx, double cy, int width, int height)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), width_(width), height_(height) {
    K_ << fx, 0, cx,
          0, fy, cy,
          0,  0,  1;
    K_inv_ << 1.0 / fx, 0, -cx / fx,
              0, 1.0 / fy, -cy / fy,
              0, 0, 1;
}

void Camera::SetDistortion(double k1, double k2, double p1, double p2, double k3) {
    k1_ = k1;
    k2_ = k2;
    p1_ = p1;
    p2_ = p2;
    k3_ = k3;
    has_distortion_ = (std::abs(k1) > 1e-12 || std::abs(k2) > 1e-12 ||
                       std::abs(p1) > 1e-12 || std::abs(p2) > 1e-12 ||
                       std::abs(k3) > 1e-12);
}

// ── Projection ────────────────────────────────────────────────────────────────

Vec2 Camera::Project(const Vec3& p_cam) const {
    if (p_cam.z() <= 1e-10) {
        return Vec2(-1, -1);
    }

    // Normalize
    const double inv_z = 1.0 / p_cam.z();
    Vec2 normalized(p_cam.x() * inv_z, p_cam.y() * inv_z);

    // Apply distortion
    if (has_distortion_) {
        normalized = Distort(normalized);
    }

    // To pixel coordinates
    return Vec2(fx_ * normalized.x() + cx_,
                fy_ * normalized.y() + cy_);
}

Vec2 Camera::ProjectWorld(const Vec3& p_w, const SE3& T_cw) const {
    const Vec3 p_cam = T_cw * p_w;
    return Project(p_cam);
}

// ── Unprojection ──────────────────────────────────────────────────────────────

Vec3 Camera::Unproject(const Vec2& pixel) const {
    // To normalized coordinates
    Vec2 normalized((pixel.x() - cx_) / fx_,
                    (pixel.y() - cy_) / fy_);

    // Undistort
    if (has_distortion_) {
        normalized = Undistort(normalized);
    }

    return Vec3(normalized.x(), normalized.y(), 1.0).normalized();
}

// ── Distortion ────────────────────────────────────────────────────────────────

Vec2 Camera::Distort(const Vec2& n) const {
    if (!has_distortion_) {
        return n;
    }

    const double x = n.x();
    const double y = n.y();
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r2 * r4;

    // Radial: (1 + k1*r^2 + k2*r^4 + k3*r^6)
    const double radial = 1.0 + k1_ * r2 + k2_ * r4 + k3_ * r6;

    // Tangential
    const double dx = 2.0 * p1_ * x * y + p2_ * (r2 + 2.0 * x * x);
    const double dy = p1_ * (r2 + 2.0 * y * y) + 2.0 * p2_ * x * y;

    return Vec2(x * radial + dx, y * radial + dy);
}

Vec2 Camera::Undistort(const Vec2& n) const {
    if (!has_distortion_) {
        return n;
    }

    // Iterative inverse of the distortion model (Gauss-Newton)
    Vec2 result = n;
    for (int i = 0; i < kMaxDistortionIterations; ++i) {
        const Vec2 distorted = Distort(result);
        const Vec2 error = distorted - n;
        if (error.norm() < kDistortionEpsilon) {
            break;
        }
        result -= error;  // Approximate Jacobian is identity
    }

    return result;
}

// ── Utilities ─────────────────────────────────────────────────────────────────

bool Camera::IsInImage(const Vec2& pixel, int border) const {
    return pixel.x() >= border && pixel.y() >= border &&
           pixel.x() < static_cast<double>(width_ - border) &&
           pixel.y() < static_cast<double>(height_ - border);
}

Mat34 Camera::ProjectionMatrix(const SE3& T_cw) const {
    Mat34 P;
    P.block<3, 3>(0, 0) = T_cw.rotation();
    P.col(3) = T_cw.translation();
    return K_ * P;
}

Camera Camera::FromParams(const CameraParams& p) {
    Camera cam(p.fx, p.fy, p.cx, p.cy, p.width, p.height);
    cam.SetDistortion(p.k1, p.k2, p.p1, p.p2, p.k3);
    return cam;
}

}  // namespace litevo
