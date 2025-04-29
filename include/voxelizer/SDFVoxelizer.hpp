#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>

namespace VXZ {

class SDFVoxelizerCPU : public VoxelizerCPU {
public:
    SDFVoxelizerCPU(){};

    // 体素化主接口
    void voxelize(VoxelGrid& grid)  override;

    // SDF函数接口
    virtual float sdf(const Eigen::Vector3f& pos) const;
};

class SDFVoxelizerGPU : public VoxelizerGPU {
public:
    SDFVoxelizerGPU(){};
    void voxelize(VoxelGrid& grid) override;
};

}
