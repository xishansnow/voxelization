#include "core/voxel_grid.hpp"
#include <cmath>

namespace VXZ {

MultiLevelVoxelGrid::MultiLevelVoxelGrid(float base_resolution,
                                       const Eigen::Vector3f& min_bounds,
                                       const Eigen::Vector3f& max_bounds,
                                       int num_levels,
                                       bool use_gpu)
    : base_resolution_(base_resolution)
    , min_bounds_(min_bounds)
    , max_bounds_(max_bounds)
    , use_gpu_(use_gpu) {
    // Create levels
    for (int i = 0; i < num_levels; ++i) {
        float resolution = base_resolution * std::pow(2.0f, i);
        levels_.push_back(std::make_unique<VoxelGrid>(resolution, min_bounds, max_bounds, use_gpu));
    }
}

const VoxelGrid& MultiLevelVoxelGrid::get_level(int level) const {
    return *levels_[level];
}

VoxelGrid& MultiLevelVoxelGrid::get_level(int level) {
    return *levels_[level];
}

int MultiLevelVoxelGrid::num_levels() const {
    return levels_.size();
}

float MultiLevelVoxelGrid::base_resolution() const {
    return base_resolution_;
}

const Eigen::Vector3f& MultiLevelVoxelGrid::min_bounds() const {
    return min_bounds_;
}

const Eigen::Vector3f& MultiLevelVoxelGrid::max_bounds() const {
    return max_bounds_;
}

bool MultiLevelVoxelGrid::use_gpu() const {
    return use_gpu_;
}

void MultiLevelVoxelGrid::update_higher_levels() {
    for (int i = 1; i < levels_.size(); ++i) {
        update_level(i);
    }
}

void MultiLevelVoxelGrid::update_level(int level) {
    const VoxelGrid& lower_level = *levels_[level - 1];
    VoxelGrid& current_level = *levels_[level];

    // For each voxel in the current level
    for (int x = 0; x < current_level.dimensions().x(); ++x) {
        for (int y = 0; y < current_level.dimensions().y(); ++y) {
            for (int z = 0; z < current_level.dimensions().z(); ++z) {
                // Get the corresponding region in the lower level
                int lower_x = x * 2;
                int lower_y = y * 2;
                int lower_z = z * 2;

                // Check if any of the 8 voxels in the lower level are filled
                bool filled = false;
                for (int dx = 0; dx < 2 && !filled; ++dx) {
                    for (int dy = 0; dy < 2 && !filled; ++dy) {
                        for (int dz = 0; dz < 2 && !filled; ++dz) {
                            if (lower_level.get(Eigen::Vector3i(lower_x + dx, lower_y + dy, lower_z + dz))) {
                                filled = true;
                            }
                        }
                    }
                }

                // Set the current level voxel
                current_level.set(Eigen::Vector3i(x, y, z), filled);
            }
        }
    }
}

} // namespace voxelization 