// =============================================================================
// LiteVO Python bindings — single-file pybind11 module
// =============================================================================
//
// Exposes the core SLAM pipeline: configuration, camera, tracker, map,
// map points, keyframes, frames, and SE(3) geometry utilities.
//
// Maps Eigen types  <-> numpy arrays (via pybind11/eigen.h)
// Maps STL containers <-> Python lists / dicts / optionals (via pybind11/stl.h)
//
// SE(3) = Eigen::Transform<double, 3, Eigen::Isometry> (Isometry3d).
// Exchanged with Python as 4x4 numpy arrays (Eigen::Matrix4d).

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>

#include "litevo/litevo.h"

namespace py = pybind11;

// ═════════════════════════════════════════════════════════════════════════════
// Internal helpers
// ═════════════════════════════════════════════════════════════════════════════

/// Convert a 2-D uint8 numpy array to cv::Mat (no copy — wraps the buffer).
/// The numpy array must remain alive while the cv::Mat is in use.
static cv::Mat ArrayToGray(py::array_t<uint8_t>& arr) {
    auto info = arr.request();
    if (info.ndim != 2) {
        throw std::runtime_error(
            "Image must be a 2‑D (H x W) uint8 numpy array, got " +
            std::to_string(info.ndim) + " dimensions");
    }
    return cv::Mat(static_cast<int>(info.shape[0]),
                   static_cast<int>(info.shape[1]),
                   CV_8UC1, info.ptr);
}

/// Convert a CV_8UC1 cv::Mat to an owning numpy array.
///
/// Descriptor accessors often return a temporary cv::Mat header; borrowing
/// its storage would leave Python with a dangling buffer once that header is
/// destroyed.  Copying the small ORB descriptor matrix keeps the Python API
/// memory-safe and also handles non-contiguous OpenCV rows.
static py::object MatToNumpy(const cv::Mat& m) {
    if (m.empty()) return py::none();
    if (m.type() != CV_8UC1 || m.dims != 2) {
        throw std::runtime_error("Expected a 2-D CV_8UC1 matrix");
    }

    py::array_t<uint8_t> result({m.rows, m.cols});
    auto* output = result.mutable_data();
    for (int row = 0; row < m.rows; ++row) {
        std::memcpy(output + static_cast<size_t>(row) * static_cast<size_t>(m.cols),
                    m.ptr<uint8_t>(row), static_cast<size_t>(m.cols));
    }
    return result;
}

/// Convert an SE3 to a 4x4 Eigen matrix (numpy-compatible).
static Eigen::Matrix4d Se3ToMat(const litevo::SE3& T) {
    return T.matrix();
}

/// Construct an SE3 from a 4x4 Eigen matrix (numpy-compatible).
static litevo::SE3 MatToSe3(const Eigen::Matrix4d& m) {
    litevo::SE3 T;
    T.matrix() = m;
    return T;
}

// ═════════════════════════════════════════════════════════════════════════════
// Module definition
// ═════════════════════════════════════════════════════════════════════════════

PYBIND11_MODULE(_litevo, m) {
    m.doc() = "LiteVO — Industrial-Grade Monocular Visual SLAM";

    // ── Version ────────────────────────────────────────────────────────────
    m.attr("__version__") = "2.0.0";

    // =========================================================================
    // TrackingState enum
    // =========================================================================

    py::enum_<litevo::tracking::TrackingState>(m, "TrackingState")
        .value("NOT_INITIALIZED", litevo::tracking::TrackingState::NOT_INITIALIZED)
        .value("INITIALIZING",    litevo::tracking::TrackingState::INITIALIZING)
        .value("OK",              litevo::tracking::TrackingState::OK)
        .value("LOST",            litevo::tracking::TrackingState::LOST)
        .export_values();

    // =========================================================================
    // ID types
    // =========================================================================

    py::class_<litevo::FrameId>(m, "FrameId")
        .def(py::init<>())
        .def(py::init<uint64_t>(), py::arg("id"))
        .def_readwrite("id", &litevo::FrameId::id)
        .def("__eq__", [](const litevo::FrameId& a, const litevo::FrameId& b) { return a == b; })
        .def("__ne__", [](const litevo::FrameId& a, const litevo::FrameId& b) { return a != b; })
        .def("__lt__", [](const litevo::FrameId& a, const litevo::FrameId& b) { return a < b; })
        .def("__hash__", [](const litevo::FrameId& f) {
            return std::hash<uint64_t>{}(f.id);
        })
        .def("__repr__", [](const litevo::FrameId& f) {
            return "FrameId(" + std::to_string(f.id) + ")";
        });

    py::class_<litevo::MapPointId>(m, "MapPointId")
        .def(py::init<>())
        .def(py::init<uint64_t>(), py::arg("id"))
        .def_readwrite("id", &litevo::MapPointId::id)
        .def("__eq__", [](const litevo::MapPointId& a, const litevo::MapPointId& b) { return a == b; })
        .def("__ne__", [](const litevo::MapPointId& a, const litevo::MapPointId& b) { return a != b; })
        .def("__lt__", [](const litevo::MapPointId& a, const litevo::MapPointId& b) { return a < b; })
        .def("__hash__", [](const litevo::MapPointId& mp) {
            return std::hash<uint64_t>{}(mp.id);
        })
        .def("__repr__", [](const litevo::MapPointId& mp) {
            return "MapPointId(" + std::to_string(mp.id) + ")";
        });

    // =========================================================================
    // Configuration structs
    // =========================================================================

    py::class_<litevo::CameraConfig>(m, "CameraConfig")
        .def(py::init<>())
        .def_readwrite("width",  &litevo::CameraConfig::width)
        .def_readwrite("height", &litevo::CameraConfig::height)
        .def_readwrite("fx",     &litevo::CameraConfig::fx)
        .def_readwrite("fy",     &litevo::CameraConfig::fy)
        .def_readwrite("cx",     &litevo::CameraConfig::cx)
        .def_readwrite("cy",     &litevo::CameraConfig::cy)
        .def_readwrite("k1",     &litevo::CameraConfig::k1)
        .def_readwrite("k2",     &litevo::CameraConfig::k2)
        .def_readwrite("p1",     &litevo::CameraConfig::p1)
        .def_readwrite("p2",     &litevo::CameraConfig::p2)
        .def_readwrite("k3",     &litevo::CameraConfig::k3)
        .def("__repr__", [](const litevo::CameraConfig& c) {
            return "<CameraConfig " + std::to_string(c.width) + "x" +
                   std::to_string(c.height) + " fx=" + std::to_string(c.fx) + ">";
        });

    py::class_<litevo::OrbConfig>(m, "OrbConfig")
        .def(py::init<>())
        .def_readwrite("num_features",  &litevo::OrbConfig::num_features)
        .def_readwrite("scale_factor",  &litevo::OrbConfig::scale_factor)
        .def_readwrite("num_levels",    &litevo::OrbConfig::num_levels)
        .def_readwrite("ini_threshold", &litevo::OrbConfig::ini_threshold)
        .def_readwrite("min_threshold", &litevo::OrbConfig::min_threshold)
        .def_readwrite("patch_size",    &litevo::OrbConfig::patch_size)
        .def("__repr__", [](const litevo::OrbConfig& o) {
            return "<OrbConfig features=" + std::to_string(o.num_features) +
                   " levels=" + std::to_string(o.num_levels) + ">";
        });

    py::class_<litevo::TrackingConfig>(m, "TrackingConfig")
        .def(py::init<>())
        .def_readwrite("min_features_for_tracking", &litevo::TrackingConfig::min_features_for_tracking)
        .def_readwrite("max_frames_between_kf",     &litevo::TrackingConfig::max_frames_between_kf)
        .def_readwrite("min_frames_between_kf",     &litevo::TrackingConfig::min_frames_between_kf)
        .def_readwrite("max_reprojection_error",    &litevo::TrackingConfig::max_reprojection_error)
        .def_readwrite("min_parallax_deg",           &litevo::TrackingConfig::min_parallax_deg);

    py::class_<litevo::MappingConfig>(m, "MappingConfig")
        .def(py::init<>())
        .def_readwrite("sliding_window_size",   &litevo::MappingConfig::sliding_window_size)
        .def_readwrite("min_observations",      &litevo::MappingConfig::min_observations)
        .def_readwrite("max_reprojection_error", &litevo::MappingConfig::max_reprojection_error);

    py::class_<litevo::LoopClosingConfig>(m, "LoopClosingConfig")
        .def(py::init<>())
        .def_readwrite("enabled",                &litevo::LoopClosingConfig::enabled)
        .def_readwrite("min_similarity_score",   &litevo::LoopClosingConfig::min_similarity_score)
        .def_readwrite("min_consecutive_loops",  &litevo::LoopClosingConfig::min_consecutive_loops)
        .def_readwrite("vocab_path",             &litevo::LoopClosingConfig::vocab_path)
        .def_readwrite("pose_graph_iterations",  &litevo::LoopClosingConfig::pose_graph_iterations)
        .def_readwrite("global_ba_iterations",   &litevo::LoopClosingConfig::global_ba_iterations)
        .def_readwrite("enable_global_ba",       &litevo::LoopClosingConfig::enable_global_ba);

    py::class_<litevo::SystemConfig>(m, "SystemConfig")
        .def(py::init<>())
        .def_readwrite("camera",          &litevo::SystemConfig::camera)
        .def_readwrite("orb",             &litevo::SystemConfig::orb)
        .def_readwrite("tracking",        &litevo::SystemConfig::tracking)
        .def_readwrite("mapping",         &litevo::SystemConfig::mapping)
        .def_readwrite("loop_closing",    &litevo::SystemConfig::loop_closing)
        .def_readwrite("input_video",      &litevo::SystemConfig::input_video)
        .def_readwrite("output_trajectory", &litevo::SystemConfig::output_trajectory)
        .def_readwrite("output_format",    &litevo::SystemConfig::output_format)
        .def_readwrite("enable_viewer",    &litevo::SystemConfig::enable_viewer)
        .def_readwrite("enable_logging",   &litevo::SystemConfig::enable_logging)
        .def_readwrite("log_level",        &litevo::SystemConfig::log_level)
        .def_static("default",       &litevo::SystemConfig::Default)
        .def_static("load_from_yaml", &litevo::SystemConfig::LoadFromYAML,
                    py::arg("path"), "Load configuration from a YAML file.\n\n"
                    "Returns std::optional<SystemConfig> — None if parsing fails.")
        .def("__repr__", [](const litevo::SystemConfig& s) {
            return "<SystemConfig camera=" + std::to_string(s.camera.width) + "x" +
                   std::to_string(s.camera.height) + ">";
        });

    // =========================================================================
    // load_config — convenience helper returning a plain Python dict
    // =========================================================================

    m.def("load_config", [](const std::string& path) -> py::dict {
        auto cfg = litevo::SystemConfig::LoadFromYAML(path);
        if (!cfg) {
            throw std::runtime_error("Failed to load config from: " + path);
        }

        py::dict cam;
        cam["width"]  = cfg->camera.width;
        cam["height"] = cfg->camera.height;
        cam["fx"]     = cfg->camera.fx;
        cam["fy"]     = cfg->camera.fy;
        cam["cx"]     = cfg->camera.cx;
        cam["cy"]     = cfg->camera.cy;
        cam["k1"]     = cfg->camera.k1;
        cam["k2"]     = cfg->camera.k2;
        cam["p1"]     = cfg->camera.p1;
        cam["p2"]     = cfg->camera.p2;
        cam["k3"]     = cfg->camera.k3;

        py::dict orb;
        orb["num_features"]  = cfg->orb.num_features;
        orb["scale_factor"]  = cfg->orb.scale_factor;
        orb["num_levels"]    = cfg->orb.num_levels;
        orb["ini_threshold"] = cfg->orb.ini_threshold;
        orb["min_threshold"] = cfg->orb.min_threshold;
        orb["patch_size"]    = cfg->orb.patch_size;

        py::dict tracking;
        tracking["min_features_for_tracking"] = cfg->tracking.min_features_for_tracking;
        tracking["max_frames_between_kf"]     = cfg->tracking.max_frames_between_kf;
        tracking["min_frames_between_kf"]     = cfg->tracking.min_frames_between_kf;
        tracking["max_reprojection_error"]    = cfg->tracking.max_reprojection_error;
        tracking["min_parallax_deg"]          = cfg->tracking.min_parallax_deg;

        py::dict mapping;
        mapping["sliding_window_size"]   = cfg->mapping.sliding_window_size;
        mapping["min_observations"]      = cfg->mapping.min_observations;
        mapping["max_reprojection_error"] = cfg->mapping.max_reprojection_error;

        py::dict loop;
        loop["enabled"]                = cfg->loop_closing.enabled;
        loop["min_similarity_score"]   = cfg->loop_closing.min_similarity_score;
        loop["min_consecutive_loops"]  = cfg->loop_closing.min_consecutive_loops;
        loop["vocab_path"]             = cfg->loop_closing.vocab_path;
        loop["pose_graph_iterations"]  = cfg->loop_closing.pose_graph_iterations;
        loop["global_ba_iterations"]   = cfg->loop_closing.global_ba_iterations;
        loop["enable_global_ba"]       = cfg->loop_closing.enable_global_ba;

        py::dict d;
        d["camera"]       = cam;
        d["orb"]          = orb;
        d["tracking"]     = tracking;
        d["mapping"]      = mapping;
        d["loop_closing"] = loop;
        d["input_video"]       = cfg->input_video;
        d["output_trajectory"] = cfg->output_trajectory;
        d["output_format"]     = cfg->output_format;
        d["enable_viewer"]     = cfg->enable_viewer;
        d["enable_logging"]    = cfg->enable_logging;
        d["log_level"]         = cfg->log_level;

        return d;
    }, py::arg("path"), "Load a YAML config file and return its contents as a dict.");

    // =========================================================================
    // Pose
    // =========================================================================

    py::class_<litevo::Pose>(m, "Pose",
        "Camera pose: position (Vec3) + orientation (Quaternion) in world frame.")
        .def(py::init<>())
        .def_readwrite("position",    &litevo::Pose::position,
                       "3-D world position (numpy float64[3])")
        .def_readwrite("orientation", &litevo::Pose::orientation,
                       "World-frame orientation quaternion (numpy float64[4]: w,x,y,z)")
        .def("to_se3", [](const litevo::Pose& p) {
            return Se3ToMat(p.ToSE3());
        }, "Convert to 4x4 SE(3) transformation matrix.")
        .def_static("from_se3", [](const Eigen::Matrix4d& m) {
            return litevo::Pose::FromSE3(MatToSe3(m));
        }, py::arg("matrix"), "Create a Pose from a 4x4 SE(3) matrix.")
        .def("__repr__", [](const litevo::Pose& p) {
            auto pos = p.position;
            return "Pose(pos=[" + std::to_string(pos.x()) + ", " +
                   std::to_string(pos.y()) + ", " +
                   std::to_string(pos.z()) + "])";
        });

    // =========================================================================
    // SE(3) geometry utilities
    // =========================================================================
    // All functions exchange SE3 as 4x4 numpy arrays (Eigen::Matrix4d).

    m.def("exp_se3", [](const litevo::Vec6& twist) {
        return Se3ToMat(litevo::geometry::ExpSE3(twist));
    }, py::arg("twist"),
    "SE(3) exponential map: 6‑D twist -> 4x4 transformation.\n"
    "twist = [vx, vy, vz, wx, wy, wz]  (translation, rotation)");

    m.def("log_se3", [](const Eigen::Matrix4d& m) {
        return litevo::geometry::LogSE3(MatToSe3(m));
    }, py::arg("matrix"),
    "SE(3) logarithm map: 4x4 transformation -> 6‑D twist.");

    m.def("make_se3_r_t", [](const litevo::Mat3& R, const litevo::Vec3& t) {
        return Se3ToMat(litevo::geometry::MakeSE3(R, t));
    }, py::arg("R"), py::arg("t"),
    "Create SE(3) from rotation matrix + translation vector.");

    m.def("make_se3_q_t", [](const litevo::Quaternion& q, const litevo::Vec3& t) {
        return Se3ToMat(litevo::geometry::MakeSE3(q, t));
    }, py::arg("q"), py::arg("t"),
    "Create SE(3) from quaternion + translation vector.");

    m.def("make_se3_aa_t", [](const litevo::Vec3& axis_angle, const litevo::Vec3& t) {
        return Se3ToMat(litevo::geometry::MakeSE3(axis_angle, t));
    }, py::arg("axis_angle"), py::arg("t"),
    "Create SE(3) from axis-angle rotation + translation vector.");

    m.def("rotation", [](const Eigen::Matrix4d& m) {
        return litevo::geometry::Rotation(MatToSe3(m));
    }, py::arg("matrix"), "Extract the 3x3 rotation matrix from an SE(3) transform.");

    m.def("translation", [](const Eigen::Matrix4d& m) {
        return litevo::geometry::Translation(MatToSe3(m));
    }, py::arg("matrix"), "Extract the 3‑D translation vector from an SE(3) transform.");

    m.def("camera_center", [](const Eigen::Matrix4d& T_cw) {
        return litevo::geometry::CameraCenter(MatToSe3(T_cw));
    }, py::arg("T_cw"), "Compute the camera center (world position) from T_cw.\n"
    "For world-to-camera pose T_cw: center = -R^T * t.");

    m.def("invert_pose", [](const Eigen::Matrix4d& T_cw) {
        return Se3ToMat(litevo::geometry::InvertPose(MatToSe3(T_cw)));
    }, py::arg("T_cw"), "Invert a world-to-camera pose -> camera-to-world.");

    m.def("relative_pose", [](const Eigen::Matrix4d& T_wc1, const Eigen::Matrix4d& T_wc2) {
        return Se3ToMat(litevo::geometry::RelativePose(MatToSe3(T_wc1), MatToSe3(T_wc2)));
    }, py::arg("T_wc1"), py::arg("T_wc2"),
    "Compute relative transformation T_rel = inv(T_wc1) * T_wc2.");

    m.def("transform_point", [](const Eigen::Matrix4d& T_cw, const litevo::Vec3& p_w) {
        return litevo::geometry::TransformPoint(MatToSe3(T_cw), p_w);
    }, py::arg("T_cw"), py::arg("p_w"),
    "Transform a 3-D point from world to camera frame.");

    m.def("transform_points", [](const Eigen::Matrix4d& T_cw_m,
                                  const std::vector<litevo::Vec3>& points_w) {
        litevo::SE3 T_cw = MatToSe3(T_cw_m);
        std::vector<litevo::Vec3> points_c;
        litevo::geometry::TransformPoints(T_cw, points_w, points_c);
        return points_c;
    }, py::arg("T_cw"), py::arg("points_w"),
    "Transform a list of 3-D points from world to camera frame.");

    m.def("adjoint_se3", [](const Eigen::Matrix4d& m) {
        return litevo::geometry::AdjointSE3(MatToSe3(m));
    }, py::arg("matrix"), "Compute the 6x6 adjoint matrix of an SE(3) element.");

    m.def("skew_symmetric", &litevo::geometry::SkewSymmetric,
          py::arg("v"), "3x3 skew-symmetric matrix of a 3‑D vector.");

    m.def("vee_operator", &litevo::geometry::VeeOperator,
          py::arg("m"), "Extract 3‑D vector from a skew-symmetric matrix (inverse of skew).");

    // =========================================================================
    // Camera
    // =========================================================================

    py::class_<litevo::Camera> cam(m, "Camera",
        "Pinhole camera model with radial-tangential distortion.");

    // CameraParams nested struct
    py::class_<litevo::Camera::CameraParams>(cam, "Params")
        .def(py::init<>())
        .def_readwrite("fx",     &litevo::Camera::CameraParams::fx)
        .def_readwrite("fy",     &litevo::Camera::CameraParams::fy)
        .def_readwrite("cx",     &litevo::Camera::CameraParams::cx)
        .def_readwrite("cy",     &litevo::Camera::CameraParams::cy)
        .def_readwrite("k1",     &litevo::Camera::CameraParams::k1)
        .def_readwrite("k2",     &litevo::Camera::CameraParams::k2)
        .def_readwrite("p1",     &litevo::Camera::CameraParams::p1)
        .def_readwrite("p2",     &litevo::Camera::CameraParams::p2)
        .def_readwrite("k3",     &litevo::Camera::CameraParams::k3)
        .def_readwrite("width",  &litevo::Camera::CameraParams::width)
        .def_readwrite("height", &litevo::Camera::CameraParams::height);

    cam.def(py::init<>())
       .def(py::init<double, double, double, double, int, int>(),
            py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"),
            py::arg("width"), py::arg("height"),
            "Construct from intrinsic parameters (no distortion).")
       .def("set_distortion", &litevo::Camera::SetDistortion,
            py::arg("k1"), py::arg("k2"), py::arg("p1"), py::arg("p2"),
            py::arg("k3") = 0.0,
            "Set radial-tangential distortion coefficients.")
       .def("project", &litevo::Camera::Project, py::arg("p_cam"),
            "Project a 3-D camera-frame point to pixel coordinates.")
       .def("project_world", [](const litevo::Camera& c, const litevo::Vec3& p_w,
                                  const Eigen::Matrix4d& T_cw_m) {
            return c.ProjectWorld(p_w, MatToSe3(T_cw_m));
        }, py::arg("p_w"), py::arg("T_cw"),
        "Project a 3-D world point to pixel coordinates given T_cw (4x4 SE3 matrix).")
       .def("unproject", &litevo::Camera::Unproject, py::arg("pixel"),
            "Unproject a pixel to a unit-norm 3-D ray in camera frame (undistorted).")
       .def("distort", &litevo::Camera::Distort, py::arg("normalized"),
            "Apply radial-tangential distortion to normalized coordinates.")
       .def("undistort", &litevo::Camera::Undistort, py::arg("normalized"),
            "Iteratively remove distortion from normalized coordinates.")
       .def("is_in_image", &litevo::Camera::IsInImage,
            py::arg("pixel"), py::arg("border") = 0,
            "Check if a pixel lies within the image bounds.")
       .def("projection_matrix", [](const litevo::Camera& c, const Eigen::Matrix4d& T_cw_m) {
            return c.ProjectionMatrix(MatToSe3(T_cw_m));
        }, py::arg("T_cw"),
        "Compute the 3x4 projection matrix P = K * [R | t] (T_cw is 4x4 SE3 matrix).")
       .def_property_readonly("K", &litevo::Camera::K,
            "3x3 intrinsic matrix.")
       .def_property_readonly("K_inv", &litevo::Camera::K_inv,
            "Inverse 3x3 intrinsic matrix.")
       .def_property_readonly("fx", &litevo::Camera::fx)
       .def_property_readonly("fy", &litevo::Camera::fy)
       .def_property_readonly("cx", &litevo::Camera::cx)
       .def_property_readonly("cy", &litevo::Camera::cy)
       .def_property_readonly("width", &litevo::Camera::width)
       .def_property_readonly("height", &litevo::Camera::height)
       .def_property_readonly("has_distortion", &litevo::Camera::has_distortion)
       .def_static("from_params", &litevo::Camera::FromParams,
                   py::arg("params"), "Create a Camera from a Camera.Params struct.")
       .def("__repr__", [](const litevo::Camera& c) {
           return "<Camera " + std::to_string(c.width()) + "x" +
                  std::to_string(c.height()) + " fx=" + std::to_string(c.fx()) + ">";
       });

    // =========================================================================
    // MapPoint
    // =========================================================================

    py::class_<litevo::MapPoint, std::shared_ptr<litevo::MapPoint>>(m, "MapPoint",
        "A 3-D landmark point observed by multiple frames.")
        .def(py::init<const litevo::Vec3&, litevo::FrameId>(),
             py::arg("position"), py::arg("reference_frame"))
        .def_property_readonly("id", &litevo::MapPoint::Id, "Unique MapPoint ID.")
        .def_property("position",
             &litevo::MapPoint::Position,
             &litevo::MapPoint::SetPosition,
             "3-D world position (thread-safe access).")
        .def_property_readonly("normal", &litevo::MapPoint::Normal,
             "Average viewing direction.")
        .def_property_readonly("reference_frame", &litevo::MapPoint::ReferenceFrame,
             "The frame that created this map point.")
        .def_property_readonly("num_observations", &litevo::MapPoint::Observations,
             "Count of frames observing this point.")
        .def_property_readonly("descriptor", [](const litevo::MapPoint& mp) -> py::object {
             return MatToNumpy(mp.Descriptor());
        }, "ORB descriptor as a numpy array, or None if not set.")
        .def("add_observation", &litevo::MapPoint::AddObservation,
             py::arg("frame_id"))
        .def("erase_observation", &litevo::MapPoint::EraseObservation,
             py::arg("frame_id"))
        .def_property("is_observed", &litevo::MapPoint::IsObserved,
                      &litevo::MapPoint::SetObserved)
        .def_property("is_found", &litevo::MapPoint::IsFound,
                      &litevo::MapPoint::SetFound)
        .def_property_readonly("found_count", &litevo::MapPoint::FoundCount)
        .def("increase_found", &litevo::MapPoint::IncreaseFound, py::arg("n") = 1)
        .def("reset_found", &litevo::MapPoint::ResetFound)
        .def_property_readonly("visible_count", &litevo::MapPoint::VisibleCount)
        .def("increase_visible", &litevo::MapPoint::IncreaseVisible, py::arg("n") = 1)
        .def("is_bad", &litevo::MapPoint::IsBad)
        .def("set_bad", &litevo::MapPoint::SetBad, py::arg("bad") = true)
        .def("get_found_ratio", &litevo::MapPoint::GetFoundRatio)
        .def("is_erase_ready", &litevo::MapPoint::IsEraseReady,
             py::arg("min_observations") = 3)
        .def("predict_scale", &litevo::MapPoint::PredictScale,
             py::arg("current_dist"), py::arg("num_levels"),
             py::arg("scale_factor"), py::arg("log_scale_factor"))
        .def("replace", &litevo::MapPoint::Replace, py::arg("other"))
        .def("__repr__", [](const litevo::MapPoint& mp) {
            auto p = mp.Position();
            return "<MapPoint id=" + std::to_string(mp.Id().id) +
                   " pos=[" + std::to_string(p.x()) + ", " +
                   std::to_string(p.y()) + ", " + std::to_string(p.z()) + "]" +
                   " obs=" + std::to_string(mp.Observations()) + ">";
        });

    // =========================================================================
    // Map
    // =========================================================================

    py::class_<litevo::Map>(m, "Map",
        "Thread-safe container for all MapPoints and KeyFrames.")
        .def(py::init<>())
        .def("add_map_point", &litevo::Map::AddMapPoint,
             py::arg("position"), py::arg("ref_frame"),
             "Create and add a new MapPoint. Returns shared_ptr to it.")
        .def("insert_map_point", &litevo::Map::InsertMapPoint, py::arg("mp"),
             "Insert a pre-existing MapPoint.")
        .def("erase_map_point", &litevo::Map::EraseMapPoint, py::arg("id"))
        .def("get_map_point", &litevo::Map::GetMapPoint, py::arg("id"),
             "Get a MapPoint by ID, or None if not found.")
        .def("get_all_map_points", &litevo::Map::GetAllMapPoints,
             "Return a list of all MapPoints.")
        .def("add_key_frame", &litevo::Map::AddKeyFrame, py::arg("kf"))
        .def("get_key_frame", &litevo::Map::GetKeyFrame, py::arg("id"),
             "Get a KeyFrame by FrameId, or None if not found.")
        .def("get_all_key_frames", &litevo::Map::GetAllKeyFrames,
             "Return a list of all KeyFrames.")
        .def("get_recent_key_frames", &litevo::Map::GetRecentKeyFrames,
             py::arg("n"), "Return the most recent N keyframes.")
        .def_property("reference_key_frame",
             &litevo::Map::ReferenceKeyFrame,
             &litevo::Map::SetReferenceKeyFrame)
        .def_property_readonly("map_point_count", &litevo::Map::MapPointCount)
        .def_property_readonly("keyframe_count",  &litevo::Map::KeyFrameCount)
        .def("clear", &litevo::Map::Clear, "Remove all MapPoints and KeyFrames.")
        .def("__repr__", [](const litevo::Map& m) {
            return "<Map points=" + std::to_string(m.MapPointCount()) +
                   " kfs=" + std::to_string(m.KeyFrameCount()) + ">";
        });

    // =========================================================================
    // Frame  (base of KeyFrame, returned by Tracker for inspection)
    // =========================================================================

    py::class_<litevo::Frame, std::shared_ptr<litevo::Frame>>(m, "Frame",
        "A single camera frame with extracted features, pose, and map-point associations.\n\n"
        "Frames are created internally by the Tracker. KeyFrame extends Frame with\n"
        "covisibility-graph capabilities.")
        .def_property_readonly("id", &litevo::Frame::Id)
        .def_property_readonly("timestamp", &litevo::Frame::Timestamp)
        .def_property_readonly("pose", [](const litevo::Frame& f) {
            return Se3ToMat(f.Pose());
        }, "Camera pose Tcw as a 4x4 numpy array.")
        .def_property_readonly("camera_center", &litevo::Frame::CameraCenter)
        .def_property_readonly("num_keypoints", &litevo::Frame::NumKeyPoints)
        .def_property_readonly("num_map_points", &litevo::Frame::NumMapPoints)
        .def("map_point_id_at", &litevo::Frame::MapPointIdAt, py::arg("idx"),
             "Get the MapPointId associated with keypoint index idx.")
        .def("get_map_point_indices", &litevo::Frame::GetMapPointIndices,
             "Return indices of all keypoints with valid MapPoint associations.")
        .def("get_features_in_area", &litevo::Frame::GetFeaturesInArea,
             py::arg("x"), py::arg("y"), py::arg("radius"),
             py::arg("min_level") = -1, py::arg("max_level") = -1,
             "Get keypoint indices within a spatial search window (undistorted coords).")
        .def_property("is_keyframe", &litevo::Frame::IsKeyFrame,
                      &litevo::Frame::SetKeyFrame)
        .def_property_readonly("descriptors", [](const litevo::Frame& f) -> py::object {
            return MatToNumpy(f.Descriptors());
        }, "ORB descriptors as a numpy array (N x 32, uint8).")
        .def("__repr__", [](const litevo::Frame& f) {
            return "<Frame id=" + std::to_string(f.Id().id) +
                   " kps=" + std::to_string(f.NumKeyPoints()) +
                   " mps=" + std::to_string(f.NumMapPoints()) +
                   (f.IsKeyFrame() ? " KF" : "") + ">";
        });

    // =========================================================================
    // KeyFrame  (extends Frame with covisibility graph)
    // =========================================================================

    py::class_<litevo::KeyFrame, std::shared_ptr<litevo::KeyFrame>,
               litevo::Frame>(m, "KeyFrame",
        "A keyframe in the SLAM map — extends Frame with a covisibility graph,\n"
        "spanning tree, and lifecycle management.")
        .def("add_connection", &litevo::KeyFrame::AddConnection,
             py::arg("kf"), py::arg("weight"))
        .def("erase_connection", &litevo::KeyFrame::EraseConnection, py::arg("kf"))
        .def("update_connections", &litevo::KeyFrame::UpdateConnections,
             py::arg("all_kfs"))
        .def("get_best_covisibility_keyframes",
             &litevo::KeyFrame::GetBestCovisibilityKeyFrames,
             py::arg("N"),
             py::return_value_policy::reference,
             "Top-N covisible keyframes by shared MapPoint weight.")
        .def("get_covisibles_by_weight",
             &litevo::KeyFrame::GetCovisiblesByWeight,
             py::arg("min_weight"),
             py::return_value_policy::reference,
             "Keyframes sharing at least min_weight MapPoints.")
        .def("get_weight", &litevo::KeyFrame::GetWeight, py::arg("kf"))
        .def_property_readonly("covisibles",
             &litevo::KeyFrame::GetCovisibleKeyFrames,
             py::return_value_policy::reference,
             "Raw covisible-keyframes map (KF* -> weight).")
        .def_property("parent",
             &litevo::KeyFrame::GetParent,
             &litevo::KeyFrame::SetParent,
             py::return_value_policy::reference)
        .def("add_child", &litevo::KeyFrame::AddChild, py::arg("child"))
        .def("erase_child", &litevo::KeyFrame::EraseChild, py::arg("child"))
        .def_property_readonly("children",
             &litevo::KeyFrame::GetChildren,
             py::return_value_policy::reference)
        .def("has_child", &litevo::KeyFrame::HasChild, py::arg("kf"))
        .def_property("is_bad", &litevo::KeyFrame::IsBad, &litevo::KeyFrame::SetBad)
        .def("get_map_point_matches", &litevo::KeyFrame::GetMapPointMatches,
             py::arg("map"),
             "Get shared_ptrs to all MapPoints observed by this keyframe.")
        .def("__repr__", [](const litevo::KeyFrame& kf) {
            return "<KeyFrame id=" + std::to_string(kf.Id().id) +
                   " kps=" + std::to_string(kf.NumKeyPoints()) + ">";
        });

    // =========================================================================
    // OrbExtractor
    // =========================================================================

    py::class_<litevo::features::OrbExtractor> orb_ext(m, "OrbExtractor",
        "Multi-scale ORB feature extractor with quadtree distribution.");

    py::class_<litevo::features::OrbExtractor::Options>(orb_ext, "Options")
        .def(py::init<>())
        .def_readwrite("num_features",  &litevo::features::OrbExtractor::Options::num_features)
        .def_readwrite("scale_factor",  &litevo::features::OrbExtractor::Options::scale_factor)
        .def_readwrite("num_levels",    &litevo::features::OrbExtractor::Options::num_levels)
        .def_readwrite("ini_threshold", &litevo::features::OrbExtractor::Options::ini_threshold)
        .def_readwrite("min_threshold", &litevo::features::OrbExtractor::Options::min_threshold)
        .def_readwrite("patch_size",    &litevo::features::OrbExtractor::Options::patch_size);

    orb_ext.def(py::init<>())
           .def(py::init<const litevo::features::OrbExtractor::Options&>(),
                py::arg("options"))
           .def("extract", [](litevo::features::OrbExtractor& ex,
                              py::array_t<uint8_t>& img) -> py::tuple {
                cv::Mat gray = ArrayToGray(img);
                std::vector<cv::KeyPoint> kps;
                cv::Mat desc;
                int n = ex.Extract(gray, kps, desc);
                // Convert keypoints to list of (x, y, size, angle, octave)
                py::list kp_list;
                for (const auto& kp : kps) {
                    kp_list.append(py::make_tuple(kp.pt.x, kp.pt.y,
                                                  kp.size, kp.angle,
                                                  kp.octave, kp.response));
                }
                return py::make_tuple(n, kp_list, MatToNumpy(desc));
           }, py::arg("image"),
           "Extract ORB features. Returns (num_features, keypoints_list, descriptors_array).\n"
           "Each keypoint is a tuple (x, y, size, angle, octave, response).")
           .def_property_readonly("scale_factors", &litevo::features::OrbExtractor::ScaleFactors)
           .def_property_readonly("inv_scale_factors", &litevo::features::OrbExtractor::InvScaleFactors)
           .def_property_readonly("level_sigmas", &litevo::features::OrbExtractor::LevelSigmas);

    // =========================================================================
    // Tracker
    // =========================================================================

    py::class_<litevo::tracking::Tracker>(m, "Tracker",
        "Monocular visual SLAM tracking frontend.\n\n"
        "Processes frames sequentially, estimating camera pose through\n"
        "initialization, motion-model tracking, and local-map refinement.\n\n"
        "IMPORTANT: Keep the Camera alive for the Tracker's lifetime — it stores\n"
        "a C++ reference, not a copy.")
        .def(py::init<const litevo::Camera&, const litevo::TrackingConfig&,
                      const litevo::OrbConfig&>(),
             py::arg("camera"), py::arg("tracking_config"), py::arg("orb_config"),
             py::keep_alive<1, 2>())  // Tracker keeps Camera alive
        .def("track", [](litevo::tracking::Tracker& t,
                         py::array_t<uint8_t>& img, double timestamp)
             -> std::optional<Eigen::Matrix4d> {
            cv::Mat gray = ArrayToGray(img);
            auto result = t.Track(gray, timestamp);
            if (result.has_value()) {
                return result->matrix();
            }
            return std::nullopt;
        }, py::arg("image"), py::arg("timestamp"),
        "Process a new frame.\n\n"
        "Args:\n"
        "    image: 2-D (H x W) uint8 numpy array (grayscale).\n"
        "    timestamp: Frame timestamp in seconds.\n\n"
        "Returns:\n"
        "    4x4 camera pose Tcw as a numpy array, or None if tracking lost.")
        .def_property_readonly("state", &litevo::tracking::Tracker::State,
             "Current tracking state (NOT_INITIALIZED / INITIALIZING / OK / LOST).")
        .def("is_initialized", &litevo::tracking::Tracker::IsInitialized)
        .def("is_lost", &litevo::tracking::Tracker::IsLost)
        .def("get_map", [](litevo::tracking::Tracker& t) -> litevo::Map& {
            return t.GetMap();
        }, py::return_value_policy::reference,
        "Get the internal Map (non-const, for inspection).")
        .def("get_const_map", [](const litevo::tracking::Tracker& t) -> const litevo::Map& {
            return t.GetMap();
        }, py::return_value_policy::reference,
        "Get a read-only view of the internal Map.")
        .def_property_readonly("num_tracked_points",
             &litevo::tracking::Tracker::NumTrackedPoints)
        .def_property_readonly("num_keyframes",
             &litevo::tracking::Tracker::NumKeyFrames)
        .def("reset", &litevo::tracking::Tracker::Reset,
             "Reset tracker to initial state.")
        .def("set_local_mapper", &litevo::tracking::Tracker::SetLocalMapper,
             py::arg("mapper"),
             "Set the local mapper for asynchronous keyframe processing (or None).")
        .def_property_readonly("local_mapper",
             &litevo::tracking::Tracker::GetLocalMapper,
             py::return_value_policy::reference,
             "Get the local mapper, or None if not set.")
        .def("__repr__", [](const litevo::tracking::Tracker& t) {
            const char* state_str = "UNKNOWN";
            switch (t.State()) {
                case litevo::tracking::TrackingState::NOT_INITIALIZED: state_str = "NOT_INITIALIZED"; break;
                case litevo::tracking::TrackingState::INITIALIZING:    state_str = "INITIALIZING";    break;
                case litevo::tracking::TrackingState::OK:              state_str = "OK";              break;
                case litevo::tracking::TrackingState::LOST:            state_str = "LOST";            break;
            }
            return "<Tracker state=" + std::string(state_str) +
                   " tracked=" + std::to_string(t.NumTrackedPoints()) +
                   " kfs=" + std::to_string(t.NumKeyFrames()) + ">";
        });

    // =========================================================================
    // LocalMapper
    // =========================================================================

    py::class_<litevo::mapping::LocalMapper>(m, "LocalMapper",
        "Background thread for map maintenance and optimization.\n\n"
        "For each new keyframe the LocalMapper:\n"
        "  1. Updates the covisibility graph\n"
        "  2. Triangulates new map points with covisible KFs\n"
        "  3. Culls bad map points\n"
        "  4. Runs local bundle adjustment\n"
        "  5. Culls redundant keyframes\n\n"
        "IMPORTANT: Keep the Map, Camera, and OrbExtractor alive for the\n"
        "LocalMapper's lifetime — it stores C++ references, not copies.")
        .def(py::init<litevo::Map&, const litevo::Camera&,
                      const litevo::MappingConfig&,
                      litevo::features::OrbExtractor&>(),
             py::arg("map"), py::arg("camera"), py::arg("config"),
             py::arg("extractor"),
             py::keep_alive<1, 2>(),  // LocalMapper keeps Map alive
             py::keep_alive<1, 3>(),  // LocalMapper keeps Camera alive
             py::keep_alive<1, 5>())  // LocalMapper keeps OrbExtractor alive
        .def("start", &litevo::mapping::LocalMapper::Start,
             "Start the mapping background thread.")
        .def("stop", &litevo::mapping::LocalMapper::Stop,
             "Stop and join the mapping thread.")
        .def("request_stop", &litevo::mapping::LocalMapper::RequestStop,
             "Request the thread to stop (non-blocking).")
        .def("is_running", &litevo::mapping::LocalMapper::IsRunning)
        .def("is_finished", &litevo::mapping::LocalMapper::IsFinished)
        .def("insert_keyframe", &litevo::mapping::LocalMapper::InsertKeyFrame,
             py::arg("kf"), "Insert a new keyframe for processing. Thread-safe.")
        .def("queue_size", &litevo::mapping::LocalMapper::QueueSize,
             "Approximate number of keyframes in queue.")
        .def_property("accept_keyframes",
             &litevo::mapping::LocalMapper::IsAcceptingKeyFrames,
             &litevo::mapping::LocalMapper::SetAcceptKeyFrames)
        .def("__repr__", [](const litevo::mapping::LocalMapper& lm) {
            return "<LocalMapper running=" + std::to_string(lm.IsRunning()) +
                   " queue=" + std::to_string(lm.QueueSize()) + ">";
        });
}
