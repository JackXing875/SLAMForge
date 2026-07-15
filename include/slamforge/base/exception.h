// =============================================================================
// SLAMForge exception hierarchy — strongly-typed error handling
// =============================================================================

#pragma once

#include <stdexcept>
#include <string>

namespace slamforge {

// ── Base exception ───────────────────────────────────────────────────────────

/// @brief Base exception class for all SLAMForge errors.
class Exception : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// ── Configuration errors ─────────────────────────────────────────────────────

/// @brief Thrown when configuration is invalid or missing required fields.
class ConfigError : public Exception {
public:
    using Exception::Exception;
};

/// @brief Thrown when a required YAML key is missing.
class MissingConfigKey : public ConfigError {
public:
    explicit MissingConfigKey(const std::string& key)
        : ConfigError("Missing required configuration key: " + key) {}
};

// ── I/O errors ───────────────────────────────────────────────────────────────

/// @brief Thrown when a file cannot be opened or read.
class IOError : public Exception {
public:
    explicit IOError(const std::string& path, const std::string& reason = "unknown")
        : Exception("I/O error on '" + path + "': " + reason) {}
};

// ── Geometry errors ──────────────────────────────────────────────────────────

/// @brief Thrown when a geometric computation fails (e.g., degenerate case).
class GeometryError : public Exception {
public:
    using Exception::Exception;
};

/// @brief Thrown when triangulation produces invalid results.
class TriangulationError : public GeometryError {
public:
    explicit TriangulationError(const std::string& reason)
        : GeometryError("Triangulation failed: " + reason) {}
};

/// @brief Thrown when PnP pose estimation fails.
class PnPError : public GeometryError {
public:
    explicit PnPError(const std::string& reason)
        : GeometryError("PnP estimation failed: " + reason) {}
};

// ── Tracking errors ──────────────────────────────────────────────────────────

/// @brief Thrown when tracking is lost and cannot recover.
class TrackingLost : public Exception {
public:
    TrackingLost() : Exception("Tracking lost — insufficient features") {}
};

// ── Optimization errors ──────────────────────────────────────────────────────

/// @brief Thrown when bundle adjustment or pose graph fails to converge.
class OptimizationError : public Exception {
public:
    using Exception::Exception;
};

// ── Not implemented ──────────────────────────────────────────────────────────

/// @brief Thrown when calling unimplemented functionality.
class NotImplemented : public Exception {
public:
    explicit NotImplemented(const std::string& feature)
        : Exception("Not yet implemented: " + feature) {}
};

}  // namespace slamforge
