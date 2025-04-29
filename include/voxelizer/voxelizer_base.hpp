#pragma once

#include "../core/voxel_grid.hpp"
#include <eigen3/Eigen/Dense>
#include <memory>

namespace VXZ {


// Base class for all voxelizers
class VoxelizerBase {
public:
    virtual ~VoxelizerBase() = default;
    
    // Main voxelization interface
    virtual VoxelGrid voxelize(float resolution,
                             const Eigen::Vector3f& min_bounds,
                             const Eigen::Vector3f& max_bounds) = 0;
    
    // Implementation method
    virtual void voxelize(VoxelGrid& grid) = 0;
};

// Base class for CPU-based voxelizers
class VoxelizerCPU : public VoxelizerBase {
public:
    virtual ~VoxelizerCPU() = default;
    
    // Main voxelization interface
    VoxelGrid voxelize(float resolution,
                      const Eigen::Vector3f& min_bounds,
                      const Eigen::Vector3f& max_bounds) override {

        VoxelGrid grid(resolution, min_bounds, max_bounds);
        voxelize(grid);

        return grid;
    }
    
    // Implementation method
    void voxelize(VoxelGrid& grid) override {};

};

// Base class for GPU-based voxelizers
class VoxelizerGPU : public VoxelizerBase {
public:
    virtual ~VoxelizerGPU() = default;
    
    // Main voxelization interface
    VoxelGrid voxelize(float resolution,
                      const Eigen::Vector3f& min_bounds,
                      const Eigen::Vector3f& max_bounds) override {
        VoxelGrid grid(resolution, min_bounds, max_bounds);
        voxelize(grid);
        return grid;
    }
        
    // GPU-specific implementation
    virtual void voxelize(VoxelGrid& grid) {};
};


} // namespace VXZ