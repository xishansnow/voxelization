#include "voxelizer/point_cloud_voxelizer.hpp"
#include <cmath>

namespace VXZ {

void PointCloudVoxelizerCPU::voxelize(VoxelGrid& grid) {
    float radius_squared = point_radius_ * point_radius_;
    
    // For each point in the cloud
    for (const auto& point : points_) {
        // Convert point to grid coordinates
        Eigen::Vector3i grid_point = grid.world_to_grid(point);
        
        // Calculate bounding box in grid coordinates
        int radius_in_voxels = std::ceil(point_radius_ / grid.resolution());
        Eigen::Vector3i grid_min = grid_point - Eigen::Vector3i::Constant(radius_in_voxels);
        Eigen::Vector3i grid_max = grid_point + Eigen::Vector3i::Constant(radius_in_voxels);
        
        // Clamp to grid bounds
        grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
        grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
        
        // Fill the sphere around the point
        for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
            for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
                for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                    Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                    float dist_squared = (world_pos - point).squaredNorm();
                    if (dist_squared <= radius_squared) {
                        grid.set(Eigen::Vector3i(x, y, z), true);
                    }
                }
            }
        }
    }
}

void PointCloudVoxelizerGPU::voxelize(VoxelGrid& grid) {
    // TODO: Implement GPU voxelization
}

} // namespace VXZ 