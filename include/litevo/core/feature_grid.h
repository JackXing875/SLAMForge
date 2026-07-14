// =============================================================================
// FeatureGrid — spatial grid for O(1) feature lookup by image region
// =============================================================================

#pragma once

#include <opencv2/core/types.hpp>

#include <vector>

namespace litevo {

/// @brief Rectangular grid dividing the image into cells for fast spatial
///        feature lookup. Each cell stores the indices of keypoints that fall
///        within its bounds.
///
/// Typical cell size is 64 pixels, giving ~20x6 cells for a 1241x376 image.
class FeatureGrid {
public:
    /// @brief Default constructor (empty grid).
    FeatureGrid() : width_(0), height_(0), cell_size_(64), grid_cols_(0), grid_rows_(0) {}

    /// @param image_width  Image width in pixels.
    /// @param image_height Image height in pixels.
    /// @param cell_size    Side length of each square cell (default 64 px).
    FeatureGrid(int image_width, int image_height, int cell_size = 64);

    /// @brief Populate grid cells with keypoint indices.
    /// Each keypoint is assigned to the cell containing its coordinates.
    void Assign(const std::vector<cv::KeyPoint>& keypoints);

    /// @brief Get all keypoint indices within a rectangular search window.
    ///
    /// Returns indices of keypoints whose undistorted coordinates lie within
    /// `radius` pixels of (x, y). Only cells overlapping the search window
    /// are examined.
    ///
    /// @param x, y      Search center (undistorted pixel coordinates).
    /// @param radius     Search radius in pixels.
    /// @param min_level  Minimum pyramid level (-1 = no restriction).
    /// @param max_level  Maximum pyramid level (-1 = no restriction).
    /// @return Vector of keypoint indices within the search window.
    std::vector<int> GetCandidates(float x, float y, float radius, int min_level = -1,
                                   int max_level = -1) const;

    /// @brief Number of grid columns.
    int GridCols() const { return grid_cols_; }

    /// @brief Number of grid rows.
    int GridRows() const { return grid_rows_; }

    /// @brief Cell size in pixels.
    int CellSize() const { return cell_size_; }

private:
    int PosToGridX(float x) const;
    int PosToGridY(float y) const;
    bool IsValidCell(int col, int row) const;

    int width_, height_, cell_size_;
    int grid_cols_, grid_rows_;

    /// grid_[row * grid_cols_ + col] = vector of keypoint indices
    std::vector<std::vector<int>> grid_;
};

}  // namespace litevo
