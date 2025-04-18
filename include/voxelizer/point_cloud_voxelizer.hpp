#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>

namespace VXZ {

// Point cloud voxelizer CPU implementation
class PointCloudVoxelizerCPU : public VoxelizerCPU {
public:
    PointCloudVoxelizerCPU(const std::vector<Eigen::Vector3f>& points,
                          float point_radius)
        : points_(points), point_radius_(point_radius) {}
    
    void voxelize(VoxelGrid& grid) override;
    
private:
    std::vector<Eigen::Vector3f> points_;
    float point_radius_;
};

// Point cloud voxelizer GPU implementation
class PointCloudVoxelizerGPU : public VoxelizerGPU {
public:
    PointCloudVoxelizerGPU(const std::vector<Eigen::Vector3f>& points,
                          float point_radius)
        : points_(points), point_radius_(point_radius) {}
    
    void voxelize(VoxelGrid& grid) override;
    
private:
    std::vector<Eigen::Vector3f> points_;
    float point_radius_;
};



} // namespace VXZ 