#include "operator/grid_operator.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <limits>

namespace VXZ {

// SmoothOperator implementation
SmoothOperator::SmoothOperator(int iterations, float threshold)
    : iterations_(iterations), threshold_(threshold) {}

bool SmoothOperator::apply(VXZ::VoxelGrid& grid) const {
    if (iterations_ <= 0) return true;

    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    // Create a temporary grid for the result
    VXZ::VoxelGrid temp_grid(width, height, depth);

    for (int iter = 0; iter < iterations_; ++iter) {
        for (int z = 0; z < depth; ++z) {
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    // Count active neighbors
                    int active_count = 0;
                    int total_count = 0;

                    for (int dz = -1; dz <= 1; ++dz) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dx = -1; dx <= 1; ++dx) {
                                int nx = x + dx;
                                int ny = y + dy;
                                int nz = z + dz;

                                if (nx >= 0 && nx < width &&
                                    ny >= 0 && ny < height &&
                                    nz >= 0 && nz < depth) {
                                    ++total_count;
                                    if (grid.get(nx, ny, nz)) {
                                        ++active_count;
                                    }
                                }
                            }
                        }
                    }

                    // Apply smoothing based on threshold
                    float ratio = static_cast<float>(active_count) / total_count;
                    temp_grid.set(x, y, z, ratio >= threshold_);
                }
            }
        }

        // Copy result back to original grid
        grid = temp_grid;
    }

    return true;
}

// DilateOperator implementation
DilateOperator::DilateOperator(int iterations)
    : iterations_(iterations) {}

bool DilateOperator::apply(VXZ::VoxelGrid& grid) const {
    if (iterations_ <= 0) return true;

    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    // Create a temporary grid for the result
    VXZ::VoxelGrid temp_grid(width, height, depth);

    for (int iter = 0; iter < iterations_; ++iter) {
        for (int z = 0; z < depth; ++z) {
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    bool has_active_neighbor = false;

                    // Check 6-connected neighbors
                    const int neighbors[6][3] = {
                        {1, 0, 0}, {-1, 0, 0},
                        {0, 1, 0}, {0, -1, 0},
                        {0, 0, 1}, {0, 0, -1}
                    };

                    for (const auto& n : neighbors) {
                        int nx = x + n[0];
                        int ny = y + n[1];
                        int nz = z + n[2];

                        if (nx >= 0 && nx < width &&
                            ny >= 0 && ny < height &&
                            nz >= 0 && nz < depth &&
                            grid.get(nx, ny, nz)) {
                            has_active_neighbor = true;
                            break;
                        }
                    }

                    temp_grid.set(x, y, z, grid.get(x, y, z) || has_active_neighbor);
                }
            }
        }

        // Copy result back to original grid
        grid = temp_grid;
    }

    return true;
}

// ErodeOperator implementation
ErodeOperator::ErodeOperator(int iterations)
    : iterations_(iterations) {}

bool ErodeOperator::apply(VXZ::VoxelGrid& grid) const {
    if (iterations_ <= 0) return true;

    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    // Create a temporary grid for the result
    VXZ::VoxelGrid temp_grid(width, height, depth);

    for (int iter = 0; iter < iterations_; ++iter) {
        for (int z = 0; z < depth; ++z) {
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    bool has_inactive_neighbor = false;

                    // Check 6-connected neighbors
                    const int neighbors[6][3] = {
                        {1, 0, 0}, {-1, 0, 0},
                        {0, 1, 0}, {0, -1, 0},
                        {0, 0, 1}, {0, 0, -1}
                    };

                    for (const auto& n : neighbors) {
                        int nx = x + n[0];
                        int ny = y + n[1];
                        int nz = z + n[2];

                        if (nx >= 0 && nx < width &&
                            ny >= 0 && ny < height &&
                            nz >= 0 && nz < depth &&
                            !grid.get(nx, ny, nz)) {
                            has_inactive_neighbor = true;
                            break;
                        }
                    }

                    temp_grid.set(x, y, z, grid.get(x, y, z) && !has_inactive_neighbor);
                }
            }
        }

        // Copy result back to original grid
        grid = temp_grid;
    }

    return true;
}

// OffsetOperator implementation
OffsetOperator::OffsetOperator(float distance)
    : distance_(distance) {}

bool OffsetOperator::apply(VXZ::VoxelGrid& grid) const {
    if (distance_ == 0.0f) return true;

    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    // Create a temporary grid for the result
    VXZ::VoxelGrid temp_grid(width, height, depth);

    // Convert distance to number of voxels
    int voxel_distance = static_cast<int>(std::ceil(std::abs(distance_)));
    bool is_positive = distance_ > 0.0f;

    for (int z = 0; z < depth; ++z) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                bool should_set = false;

                // Check all voxels within the distance
                for (int dz = -voxel_distance; dz <= voxel_distance; ++dz) {
                    for (int dy = -voxel_distance; dy <= voxel_distance; ++dy) {
                        for (int dx = -voxel_distance; dx <= voxel_distance; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            int nz = z + dz;

                            if (nx >= 0 && nx < width &&
                                ny >= 0 && ny < height &&
                                nz >= 0 && nz < depth) {
                                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                                if (dist <= std::abs(distance_) && grid.get(nx, ny, nz)) {
                                    should_set = true;
                                    break;
                                }
                            }
                        }
                        if (should_set) break;
                    }
                    if (should_set) break;
                }

                temp_grid.set(x, y, z, is_positive ? should_set : !should_set);
            }
        }
    }

    // Copy result back to original grid
    grid = temp_grid;
    return true;
}

// UnionOperator implementation
UnionOperator::UnionOperator(const VXZ::VoxelGrid& other)
    : other_(other) {}

bool UnionOperator::apply(VXZ::VoxelGrid& grid) const {
    if (grid.get_width() != other_.get_width() ||
        grid.get_height() != other_.get_height() ||
        grid.get_depth() != other_.get_depth()) {
        return false;
    }

    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    for (int z = 0; z < depth; ++z) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                grid.set(x, y, z, grid.get(x, y, z) || other_.get(x, y, z));
            }
        }
    }

    return true;
}

// IntersectionOperator implementation
IntersectionOperator::IntersectionOperator(const VXZ::VoxelGrid& other)
    : other_(other) {}

bool IntersectionOperator::apply(VXZ::VoxelGrid& grid) const {
    if (grid.get_width() != other_.get_width() ||
        grid.get_height() != other_.get_height() ||
        grid.get_depth() != other_.get_depth()) {
        return false;
    }

    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    for (int z = 0; z < depth; ++z) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                grid.set(x, y, z, grid.get(x, y, z) && other_.get(x, y, z));
            }
        }
    }

    return true;
}

// DifferenceOperator implementation
DifferenceOperator::DifferenceOperator(const VXZ::VoxelGrid& other)
    : other_(other) {}

bool DifferenceOperator::apply(VXZ::VoxelGrid& grid) const {
    if (grid.get_width() != other_.get_width() ||
        grid.get_height() != other_.get_height() ||
        grid.get_depth() != other_.get_depth()) {
        return false;
    }

    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    for (int z = 0; z < depth; ++z) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                grid.set(x, y, z, grid.get(x, y, z) && !other_.get(x, y, z));
            }
        }
    }

    return true;
}

// OpeningOperator implementation
OpeningOperator::OpeningOperator(int iterations)
    : iterations_(iterations) {}

bool OpeningOperator::apply(VXZ::VoxelGrid& grid) const {
    if (iterations_ <= 0) return true;

    // First apply erosion
    ErodeOperator erode(iterations_);
    if (!erode.apply(grid)) return false;

    // Then apply dilation
    DilateOperator dilate(iterations_);
    return dilate.apply(grid);
}

// ClosingOperator implementation
ClosingOperator::ClosingOperator(int iterations)
    : iterations_(iterations) {}

bool ClosingOperator::apply(VXZ::VoxelGrid& grid) const {
    if (iterations_ <= 0) return true;

    // First apply dilation
    DilateOperator dilate(iterations_);
    if (!dilate.apply(grid)) return false;

    // Then apply erosion
    ErodeOperator erode(iterations_);
    return erode.apply(grid);
}

// DistanceTransformOperator implementation
DistanceTransformOperator::DistanceTransformOperator(float max_distance)
    : max_distance_(max_distance) {}

bool DistanceTransformOperator::apply(VXZ::VoxelGrid& grid) const {
    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    // Create a temporary grid for distances
    VXZ::VoxelGrid distance_grid(width, height, depth);
    std::queue<std::tuple<int, int, int, float>> queue;

    // Initialize distances
    for (int z = 0; z < depth; ++z) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (grid.get(x, y, z)) {
                    distance_grid.set(x, y, z, 0.0f);
                    queue.push({x, y, z, 0.0f});
                } else {
                    distance_grid.set(x, y, z, max_distance_);
                }
            }
        }
    }

    // Process queue
    while (!queue.empty()) {
        auto [x, y, z, dist] = queue.front();
        queue.pop();

        // Check 6-connected neighbors
        const int neighbors[6][3] = {
            {1, 0, 0}, {-1, 0, 0},
            {0, 1, 0}, {0, -1, 0},
            {0, 0, 1}, {0, 0, -1}
        };

        for (const auto& n : neighbors) {
            int nx = x + n[0];
            int ny = y + n[1];
            int nz = z + n[2];

            if (nx >= 0 && nx < width &&
                ny >= 0 && ny < height &&
                nz >= 0 && nz < depth) {
                float new_dist = dist + 1.0f;
                if (new_dist < distance_grid.get(nx, ny, nz)) {
                    distance_grid.set(nx, ny, nz, new_dist);
                    queue.push({nx, ny, nz, new_dist});
                }
            }
        }
    }

    // Copy result back to original grid
    grid = distance_grid;
    return true;
}

// ConnectedComponentsOperator implementation
ConnectedComponentsOperator::ConnectedComponentsOperator(int connectivity)
    : connectivity_(connectivity), component_count_(0) {}

bool ConnectedComponentsOperator::apply(VXZ::VoxelGrid& grid) const {
    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    // Create a temporary grid for labels
    VXZ::VoxelGrid label_grid(width, height, depth);
    std::queue<std::tuple<int, int, int>> queue;
    int current_label = 1;

    // Initialize labels
    for (int z = 0; z < depth; ++z) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (grid.get(x, y, z) && label_grid.get(x, y, z) == 0) {
                    // Start new component
                    label_grid.set(x, y, z, current_label);
                    queue.push({x, y, z});

                    // Process component
                    while (!queue.empty()) {
                        auto [cx, cy, cz] = queue.front();
                        queue.pop();

                        // Check neighbors based on connectivity
                        for (int dz = -1; dz <= 1; ++dz) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                for (int dx = -1; dx <= 1; ++dx) {
                                    if (dx == 0 && dy == 0 && dz == 0) continue;
                                    if (connectivity_ == 6 && 
                                        !(dx == 0 || dy == 0 || dz == 0)) continue;

                                    int nx = cx + dx;
                                    int ny = cy + dy;
                                    int nz = cz + dz;

                                    if (nx >= 0 && nx < width &&
                                        ny >= 0 && ny < height &&
                                        nz >= 0 && nz < depth &&
                                        grid.get(nx, ny, nz) &&
                                        label_grid.get(nx, ny, nz) == 0) {
                                        label_grid.set(nx, ny, nz, current_label);
                                        queue.push({nx, ny, nz});
                                    }
                                }
                            }
                        }
                    }

                    ++current_label;
                }
            }
        }
    }

    // Copy result back to original grid
    grid = label_grid;
    component_count_ = current_label - 1;
    return true;
}

int ConnectedComponentsOperator::get_component_count() const {
    return component_count_;
}

// FillOperator implementation
FillOperator::FillOperator(const Eigen::Vector3i& seed_point, int connectivity)
    : seed_point_(seed_point), connectivity_(connectivity) {}

bool FillOperator::apply(VXZ::VoxelGrid& grid) const {
    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    // Check if seed point is valid
    if (seed_point_.x() < 0 || seed_point_.x() >= width ||
        seed_point_.y() < 0 || seed_point_.y() >= height ||
        seed_point_.z() < 0 || seed_point_.z() >= depth) {
        return false;
    }

    // Create a temporary grid for the result
    VXZ::VoxelGrid result_grid(width, height, depth);
    std::queue<std::tuple<int, int, int>> queue;

    // Start from seed point
    result_grid.set(seed_point_.x(), seed_point_.y(), seed_point_.z(), true);
    queue.push({seed_point_.x(), seed_point_.y(), seed_point_.z()});

    // Process queue
    while (!queue.empty()) {
        auto [x, y, z] = queue.front();
        queue.pop();

        // Check neighbors based on connectivity
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    if (connectivity_ == 6 && 
                        !(dx == 0 || dy == 0 || dz == 0)) continue;

                    int nx = x + dx;
                    int ny = y + dy;
                    int nz = z + dz;

                    if (nx >= 0 && nx < width &&
                        ny >= 0 && ny < height &&
                        nz >= 0 && nz < depth &&
                        !grid.get(nx, ny, nz) &&
                        !result_grid.get(nx, ny, nz)) {
                        result_grid.set(nx, ny, nz, true);
                        queue.push({nx, ny, nz});
                    }
                }
            }
        }
    }

    // Copy result back to original grid
    grid = result_grid;
    return true;
}

// InterpolationOperator implementation
InterpolationOperator::InterpolationOperator(const Eigen::Vector3f& position)
    : position_(position), interpolated_value_(0.0f) {}

float InterpolationOperator::get_value() const {
    return interpolated_value_;
}

bool InterpolationOperator::apply(VXZ::VoxelGrid& grid) const {
    int width = grid.get_width();
    int height = grid.get_height();
    int depth = grid.get_depth();

    // Check if position is within grid bounds
    if (position_.x() < 0.0f || position_.x() >= width ||
        position_.y() < 0.0f || position_.y() >= height ||
        position_.z() < 0.0f || position_.z() >= depth) {
        return false;
    }

    // Get the 8 surrounding voxels
    int x0 = static_cast<int>(std::floor(position_.x()));
    int y0 = static_cast<int>(std::floor(position_.y()));
    int z0 = static_cast<int>(std::floor(position_.z()));
    int x1 = std::min(x0 + 1, width - 1);
    int y1 = std::min(y0 + 1, height - 1);
    int z1 = std::min(z0 + 1, depth - 1);

    // Get the values at the 8 corners
    float v000 = grid.get(x0, y0, z0);
    float v001 = grid.get(x0, y0, z1);
    float v010 = grid.get(x0, y1, z0);
    float v011 = grid.get(x0, y1, z1);
    float v100 = grid.get(x1, y0, z0);
    float v101 = grid.get(x1, y0, z1);
    float v110 = grid.get(x1, y1, z0);
    float v111 = grid.get(x1, y1, z1);

    // Calculate interpolation weights
    float xd = position_.x() - x0;
    float yd = position_.y() - y0;
    float zd = position_.z() - z0;

    // Perform trilinear interpolation
    float c00 = v000 * (1.0f - xd) + v100 * xd;
    float c01 = v001 * (1.0f - xd) + v101 * xd;
    float c10 = v010 * (1.0f - xd) + v110 * xd;
    float c11 = v011 * (1.0f - xd) + v111 * xd;

    float c0 = c00 * (1.0f - yd) + c10 * yd;
    float c1 = c01 * (1.0f - yd) + c11 * yd;

    interpolated_value_ = c0 * (1.0f - zd) + c1 * zd;
    return true;
}

} // namespace VXZ 