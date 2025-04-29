#include "voxelizer/point_cloud_voxelizer.hpp"
#include <cmath>

namespace VXZ {

void PointCloudVoxelizerCPU::voxelize(VoxelGrid& grid) {
    
    // 计算点云的边界框
    Eigen::Vector3f min = points_[0];
    Eigen::Vector3f max = points_[0];
    
    for (const auto& point : points_) {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point); 
    }
    
    // 转换为网格坐标
    Eigen::Vector3i grid_min = grid.world_to_grid(min);
    Eigen::Vector3i grid_max = grid.world_to_grid(max);
    
    // 限制在网格边界内
    grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
    grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
    
    // 遍历每个点
    float radius_squared = point_radius_ * point_radius_;
    for (const auto& point : points_) {
        // 转换为网格坐标
        Eigen::Vector3i grid_point = grid.world_to_grid(point);
        
        // 计算点周围的体素范围
        int radius_voxels = std::ceil(point_radius_ / grid.resolution());
        Eigen::Vector3i local_min = grid_point - Eigen::Vector3i::Constant(radius_voxels);
        Eigen::Vector3i local_max = grid_point + Eigen::Vector3i::Constant(radius_voxels);
        
        // 限制在网格边界内
        local_min = local_min.cwiseMax(grid_min);
        local_max = local_max.cwiseMin(grid_max);
        
        // 填充点周围的球形区域
        for (int x = local_min.x(); x <= local_max.x(); ++x) {
            for (int y = local_min.y(); y <= local_max.y(); ++y) {
                for (int z = local_min.z(); z <= local_max.z(); ++z) {
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