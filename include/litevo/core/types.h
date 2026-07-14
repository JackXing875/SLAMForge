// =============================================================================
// LiteVO core types — fundamental data structures and type aliases
// =============================================================================

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <vector>

namespace litevo {

// ── Scalar type ──────────────────────────────────────────────────────────────

/// Default scalar type for all floating-point computations.
using Scalar = double;

// ── Vector type aliases ──────────────────────────────────────────────────────

using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Vec4 = Eigen::Matrix<Scalar, 4, 1>;
using Vec6 = Eigen::Matrix<Scalar, 6, 1>;

using Vec2f = Eigen::Matrix<float, 2, 1>;
using Vec3f = Eigen::Matrix<float, 3, 1>;
using Vec2i = Eigen::Matrix<int, 2, 1>;

// ── Matrix type aliases ──────────────────────────────────────────────────────

using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
using Mat4 = Eigen::Matrix<Scalar, 4, 4>;
using Mat6 = Eigen::Matrix<Scalar, 6, 6>;
using Mat34 = Eigen::Matrix<Scalar, 3, 4>;
using Mat36 = Eigen::Matrix<Scalar, 3, 6>;
using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

// ── Quaternion ───────────────────────────────────────────────────────────────

using Quaternion = Eigen::Quaternion<Scalar>;

// ── SE(3) transform ──────────────────────────────────────────────────────────

/// @brief Rigid 3D transformation (rotation + translation).
///
/// Represented as a 4x4 homogeneous matrix:
///   [ R  t ]
///   [ 0  1 ]
/// where R is a 3x3 rotation matrix and t is a 3x1 translation vector.
using SE3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

// ── Pose (for convenience) ───────────────────────────────────────────────────

/// @brief Camera pose: position and orientation in world frame.
struct Pose {
    Vec3 position{0.0, 0.0, 0.0};
    Quaternion orientation{Quaternion::Identity()};

    /// Convert to 4x4 transformation matrix.
    SE3 ToSE3() const {
        SE3 T = SE3::Identity();
        T.translation() = position;
        T.linear() = orientation.toRotationMatrix();
        return T;
    }

    /// Create from SE3.
    static Pose FromSE3(const SE3& T) {
        Pose p;
        p.position = T.translation();
        p.orientation = Quaternion(T.rotation());
        return p;
    }
};

// ── Image / feature types ────────────────────────────────────────────────────

/// @brief A detected 2D image feature point.
struct Feature {
    Vec2 pixel;          ///< Pixel coordinates (u, v)
    float response = 0;  ///< Corner response strength
    int octave = 0;      ///< Pyramid octave level
    float angle = 0;     ///< Orientation in radians

    /// Optional: associated 3D map point ID (-1 = none)
    int64_t map_point_id = -1;
};

/// @brief Reference to a frame (image with features and pose).
struct FrameId {
    uint64_t id = 0;

    bool operator==(const FrameId& other) const { return id == other.id; }
    bool operator!=(const FrameId& other) const { return id != other.id; }
    bool operator<(const FrameId& other) const { return id < other.id; }
};

/// @brief Reference to a 3D map point.
struct MapPointId {
    uint64_t id = 0;

    bool operator==(const MapPointId& other) const { return id == other.id; }
    bool operator!=(const MapPointId& other) const { return id != other.id; }
    bool operator<(const MapPointId& other) const { return id < other.id; }
};

// Hash functions for unordered containers.
struct FrameIdHash {
    std::size_t operator()(FrameId fid) const noexcept { return std::hash<uint64_t>{}(fid.id); }
};

struct MapPointIdHash {
    std::size_t operator()(MapPointId mpid) const noexcept {
        return std::hash<uint64_t>{}(mpid.id);
    }
};

}  // namespace litevo
