// =============================================================================
// SLAMForge map export
// =============================================================================

#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

namespace slamforge {

class Map;

namespace io {

/// Result returned by a map export operation.
struct MapExportResult {
    bool success = false;
    std::size_t point_count = 0;
    std::string error;

    explicit operator bool() const noexcept { return success; }
};

/// Export the valid sparse landmarks in a map as an ASCII PLY point cloud.
///
/// The function takes a coherent snapshot under the map graph lock, skips bad
/// and non-finite landmarks, then releases the lock before writing the file.
/// Existing files are replaced only after the output stream is opened.
[[nodiscard]] MapExportResult ExportMapAsPly(const Map& map,
                                             const std::filesystem::path& output_path);

}  // namespace io
}  // namespace slamforge
