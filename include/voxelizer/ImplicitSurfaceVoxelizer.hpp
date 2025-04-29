#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>


namespace VXZ {

class ImplicitSurfaceVoxelizerCPU : public VoxelizerCPU {
public:
    ImplicitSurfaceVoxelizerCPU(){};

    // 体素化主接口
    void voxelize(VoxelGrid& grid) override;

    // 隐式面函数接口
    virtual float implicit_function(const Eigen::Vector3f& pos) const;
};

class ImplicitSurfaceVoxelizerGPU : public VoxelizerGPU {
public:
    ImplicitSurfaceVoxelizerGPU(){};
    void voxelize(VoxelGrid& grid) override;
};




}
