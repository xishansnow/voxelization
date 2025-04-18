#pragma once

#include "core/voxel_grid.hpp"
#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>

namespace VXZ {

// Sphere voxelizer CPU implementation
class SphereVoxelizerCPU : public VoxelizerCPU {
public:
    SphereVoxelizerCPU(const Eigen::Vector3f& center, float radius)
        : center_(center), radius_(radius) {}
    
    void voxelize(VoxelGrid& grid) override;
    
private:
    Eigen::Vector3f center_;
    float radius_;
};

// Sphere voxelizer GPU implementation
class SphereVoxelizerGPU : public VoxelizerGPU {
public:
    SphereVoxelizerGPU(const Eigen::Vector3f& center, float radius)
        : center_(center), radius_(radius) {}
    
    void voxelize(VoxelGrid& grid) override;
    
private:
    Eigen::Vector3f center_;
    float radius_;
};



} // namespace VXZ 