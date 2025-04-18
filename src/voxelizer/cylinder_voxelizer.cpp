#include "voxelizer/cylinder_voxelizer.hpp"
#include <cmath>

namespace VXZ {

void CylinderVoxelizerCPU::voxelize(VoxelGrid& grid) {
    
    float radius_squared = radius_ * radius_;
    float half_height = height_ / 2.0f;
    
    // Calculate cylinder endpoints
    Eigen::Vector3f start = center_ - axis_ * half_height;
    Eigen::Vector3f end = center_ + axis_ * half_height;
    
    // Convert to grid coordinates
    Eigen::Vector3i grid_start = grid.world_to_grid(start);
    Eigen::Vector3i grid_end = grid.world_to_grid(end);
    
    // Calculate bounding box in grid coordinates
    int radius_in_voxels = std::ceil(radius_ / grid.resolution());
    Eigen::Vector3i grid_min = grid_start.cwiseMin(grid_end) - Eigen::Vector3i::Constant(radius_in_voxels);
    Eigen::Vector3i grid_max = grid_start.cwiseMax(grid_end) + Eigen::Vector3i::Constant(radius_in_voxels);
    
    // Clamp to grid bounds
    grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
    grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
    
    // Fill the cylinder
    for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
        for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
            for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                
                // Project point onto cylinder axis
                Eigen::Vector3f to_point = world_pos - start;
                float projection = to_point.dot(axis_);
                
                // Check if point is within height bounds
                if (projection < 0.0f || projection > height_) {
                    continue;
                }
                
                // Calculate distance from axis
                Eigen::Vector3f point_on_axis = start + axis_ * projection;
                float dist_squared = (world_pos - point_on_axis).squaredNorm();
                
                if (dist_squared <= radius_squared) {
                    grid.set(Eigen::Vector3i(x, y, z), true);
                }
            }
        }
    }
}

} // namespace VXZ