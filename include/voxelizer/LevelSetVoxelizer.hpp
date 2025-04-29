#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>

namespace VXZ {

class LevelSetVoxelizerCPU : public VoxelizerCPU {
public:
    LevelSetVoxelizerCPU(){};
    // 体素化主接口
    void voxelize(VoxelGrid& grid) override;
    

    // 可扩展的 level set 函数接口
    virtual float level_set_function(const Eigen::Vector3f& pos) const;
};

class LevelSetVoxelizerGPU : public VoxelizerGPU {
public:
    LevelSetVoxelizerGPU(){};
    void voxelize(VoxelGrid& grid) override;
};

}
