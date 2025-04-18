#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>

namespace VXZ {

// Box voxelizer implementation
class BoxVoxelizerCPU : public VoxelizerCPU {
public:
    BoxVoxelizerCPU(const Eigen::Vector3f& center, const Eigen::Vector3f& size)
        : center_(center), size_(size) {}
    
    void voxelize(VoxelGrid& grid) override;
    
private:
    Eigen::Vector3f center_;
    Eigen::Vector3f size_;
};

class BoxVoxelizerGPU : public VoxelizerGPU {
public:
    BoxVoxelizerGPU(const Eigen::Vector3f& center, const Eigen::Vector3f& size)
        : center_(center), size_(size) {}
    
    void voxelize(VoxelGrid& grid) override;        
    
private:
    Eigen::Vector3f center_;
    Eigen::Vector3f size_;
};


} // namespace VXZ 