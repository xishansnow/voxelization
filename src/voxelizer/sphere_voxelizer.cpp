#include "voxelizer/sphere_voxelizer.hpp"
#include <cmath>

namespace VXZ {

void SphereVoxelizerCPU::voxelize(VoxelGrid& grid) {
    float radius_squared = radius_ * radius_;
    
    // Convert center to grid coordinates
    Eigen::Vector3i grid_center = grid.world_to_grid(center_);
    
    // Calculate bounding box in grid coordinates
    int radius_in_voxels = std::ceil(radius_ / grid.resolution());
    Eigen::Vector3i grid_min = grid_center - Eigen::Vector3i::Constant(radius_in_voxels);
    Eigen::Vector3i grid_max = grid_center + Eigen::Vector3i::Constant(radius_in_voxels);
    
    // Clamp to grid bounds
    grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
    grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
    
    // Fill the sphere
    for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
        for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
            for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                float dist_squared = (world_pos - center_).squaredNorm();
                if (dist_squared <= radius_squared) {
                    grid.set(Eigen::Vector3i(x, y, z), true);
                }
            }
        }
    }
}

void SphereVoxelizerGPU::voxelize(VoxelGrid& grid) {
        float radius_squared = radius_ * radius_;
        
        // Convert center to grid coordinates
        Eigen::Vector3i grid_center = grid.world_to_grid(center_);
        
        // Calculate bounding box in grid coordinates
        int radius_in_voxels = std::ceil(radius_ / grid.resolution());
        Eigen::Vector3i grid_min = grid_center - Eigen::Vector3i::Constant(radius_in_voxels);
        Eigen::Vector3i grid_max = grid_center + Eigen::Vector3i::Constant(radius_in_voxels);
        
        // Clamp to grid bounds
        grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
        grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
        
        // GPU implementation would use CUDA or other parallel processing here
        // For now, we'll use the same algorithm as CPU but in the future this should be replaced
        for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
            for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
                for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                    Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                    float dist_squared = (world_pos - center_).squaredNorm();
                    if (dist_squared <= radius_squared) {
                    grid.set(Eigen::Vector3i(x, y, z), true);
                }
            }
        }
    }
}

} // namespace VXZ 