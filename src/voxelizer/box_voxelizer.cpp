#include "voxelizer/box_voxelizer.hpp"
#include <algorithm>

namespace VXZ {
// Implementation of BoxVoxelizerCPU::voxelize
void BoxVoxelizerCPU::voxelize(VoxelGrid& grid) {
    // Get half size of the box
    const Eigen::Vector3f half_size = size_ * 0.5f;
    const Eigen::Vector3f min_point = center_ - half_size;
    const Eigen::Vector3f max_point = center_ + half_size;
    
    // Get grid dimensions and properties
    const Eigen::Vector3i& dims = grid.dimensions();
    const float resolution = grid.resolution();
    const Eigen::Vector3f& origin = grid.origin();
    
    // Convert box bounds to voxel coordinates
    Eigen::Vector3i min_voxel = ((min_point - origin) / resolution).cast<int>();
    Eigen::Vector3i max_voxel = ((max_point - origin) / resolution).cast<int>();
    
    // Clamp to grid bounds
    min_voxel = min_voxel.cwiseMax(Eigen::Vector3i(0, 0, 0));
    max_voxel = max_voxel.cwiseMin(dims - Eigen::Vector3i(1, 1, 1));
    
    // Set voxels inside the box
    for (int z = min_voxel.z(); z <= max_voxel.z(); ++z) {
        for (int y = min_voxel.y(); y <= max_voxel.y(); ++y) {
            for (int x = min_voxel.x(); x <= max_voxel.x(); ++x) {
                 grid.set(Eigen::Vector3i(x, y, z), true);              
            }
        }
    }
}

// Implementation of BoxVoxelizerCPU::voxelize
void BoxVoxelizerGPU::voxelize(VoxelGrid& grid) {
    const Eigen::Vector3f half_size = size_ * 0.5f;
    const Eigen::Vector3f min_point = center_ - half_size;
    const Eigen::Vector3f max_point = center_ + half_size;
    
    // Get grid dimensions and properties
    const Eigen::Vector3i& dims = grid.dimensions();
    const float resolution = grid.resolution();
    const Eigen::Vector3f& origin = grid.origin();
    
    // Convert box bounds to voxel coordinates
    Eigen::Vector3i min_voxel = ((min_point - origin) / resolution).cast<int>();
    Eigen::Vector3i max_voxel = ((max_point - origin) / resolution).cast<int>();
    
    // Clamp to grid bounds
    min_voxel = min_voxel.cwiseMax(Eigen::Vector3i(0, 0, 0));
    max_voxel = max_voxel.cwiseMin(dims - Eigen::Vector3i(1, 1, 1));
    
    // Set voxels inside the box
    for (int z = min_voxel.z(); z <= max_voxel.z(); ++z) {
        for (int y = min_voxel.y(); y <= max_voxel.y(); ++y) {
            for (int x = min_voxel.x(); x <= max_voxel.x(); ++x) {
                grid.set(Eigen::Vector3i(x, y, z), true);
            }
        }
    }
}

} // namespace VXZ