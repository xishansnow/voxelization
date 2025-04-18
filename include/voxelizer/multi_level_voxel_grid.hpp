#pragma once

#include "../core/voxel_grid.hpp"
#include <vector>
#include <memory>

namespace VXZ {

class MultiLevelVoxelGrid {
public:
    MultiLevelVoxelGrid(float base_resolution,
                       const Eigen::Vector3f& min_bounds,
                       const Eigen::Vector3f& max_bounds,
                       int num_levels);

    // Get voxel grid at specific level
    const VoxelGrid& get_level(int level) const;
    VoxelGrid& get_level(int level);

    // Get number of levels
    int num_levels() const;

    // Get base resolution
    float base_resolution() const;

    // Get bounds
    const Eigen::Vector3f& min_bounds() const;
    const Eigen::Vector3f& max_bounds() const;

    // Update higher level grids based on lower level
    void update_higher_levels();

private:
    float base_resolution_;
    Eigen::Vector3f min_bounds_;
    Eigen::Vector3f max_bounds_;
    std::vector<std::unique_ptr<VoxelGrid>> levels_;

    // Update a specific level based on the level below
    void update_level(int level);
};

} // namespace voxelization 