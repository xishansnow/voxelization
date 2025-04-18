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
    
    void voxelize_cpu(VoxelGrid& grid) override;
    
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
    
    void voxelize_gpu(VoxelGrid& grid) override;
    
private:
    std::vector<Eigen::Vector3f> points_;
    float point_radius_;
};

// Factory class for PointCloudVoxelizer
class PointCloudVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(const std::vector<Eigen::Vector3f>& points,
                                               float point_radius,
                                               bool use_gpu = false) {
        if (use_gpu) {
            return std::make_unique<PointCloudVoxelizerGPU>(points, point_radius);
        } else {
            return std::make_unique<PointCloudVoxelizerCPU>(points, point_radius);
        }
    }
};

} // namespace voxelization 