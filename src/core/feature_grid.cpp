// =============================================================================
// FeatureGrid implementation
// =============================================================================

#include "litevo/core/feature_grid.h"

#include <algorithm>
#include <cmath>

namespace litevo {

FeatureGrid::FeatureGrid(int image_width, int image_height, int cell_size)
    : width_(image_width), height_(image_height), cell_size_(cell_size) {
    grid_cols_ = static_cast<int>(std::ceil(static_cast<float>(width_) / cell_size_));
    grid_rows_ = static_cast<int>(std::ceil(static_cast<float>(height_) / cell_size_));
    grid_.resize(static_cast<size_t>(grid_cols_ * grid_rows_));
}

void FeatureGrid::Assign(const std::vector<cv::KeyPoint>& keypoints) {
    // Clear all cells
    for (auto& cell : grid_) {
        cell.clear();
    }

    for (size_t i = 0; i < keypoints.size(); ++i) {
        int col = PosToGridX(keypoints[i].pt.x);
        int row = PosToGridY(keypoints[i].pt.y);
        if (IsValidCell(col, row)) {
            grid_[static_cast<size_t>(row * grid_cols_ + col)].push_back(static_cast<int>(i));
        }
    }
}

std::vector<int> FeatureGrid::GetCandidates(float x, float y, float radius, int min_level,
                                            int max_level) const {
    std::vector<int> candidates;

    // Determine cell range to search
    int min_col = PosToGridX(x - radius);
    int max_col = PosToGridX(x + radius);
    int min_row = PosToGridY(y - radius);
    int max_row = PosToGridY(y + radius);

    // Clamp to valid range
    min_col = std::max(0, min_col);
    max_col = std::min(grid_cols_ - 1, max_col);
    min_row = std::max(0, min_row);
    max_row = std::min(grid_rows_ - 1, max_row);

    for (int row = min_row; row <= max_row; ++row) {
        for (int col = min_col; col <= max_col; ++col) {
            const auto& cell = grid_[static_cast<size_t>(row * grid_cols_ + col)];
            candidates.insert(candidates.end(), cell.begin(), cell.end());
        }
    }

    (void)min_level;
    (void)max_level;

    return candidates;
}

int FeatureGrid::PosToGridX(float x) const {
    return static_cast<int>(std::floor(x / cell_size_));
}

int FeatureGrid::PosToGridY(float y) const {
    return static_cast<int>(std::floor(y / cell_size_));
}

bool FeatureGrid::IsValidCell(int col, int row) const {
    return col >= 0 && col < grid_cols_ && row >= 0 && row < grid_rows_;
}

}  // namespace litevo
