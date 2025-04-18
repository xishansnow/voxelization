#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>

namespace VXZ {

// Sphere voxelizer CPU implementation
class SphereVoxelizerCPU : public VoxelizerCPU {
public:
    SphereVoxelizerCPU(const Eigen::Vector3f& center, float radius)
        : center_(center), radius_(radius) {}
    
    void voxelize_cpu(VoxelGrid& grid) override;
    
private:
    Eigen::Vector3f center_;
    float radius_;
};

// Sphere voxelizer GPU implementation
class SphereVoxelizerGPU : public VoxelizerGPU {
public:
    SphereVoxelizerGPU(const Eigen::Vector3f& center, float radius)
        : center_(center), radius_(radius) {}
    
    void voxelize_gpu(VoxelGrid& grid) override;
    
private:
    Eigen::Vector3f center_;
    float radius_;
};

// Factory class for SphereVoxelizer
class SphereVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(const Eigen::Vector3f& center,
                                               float radius,
                                               bool use_gpu = false) {
        if (use_gpu) {
            return std::make_unique<SphereVoxelizerGPU>(center, radius);
        } else {
            return std::make_unique<SphereVoxelizerCPU>(center, radius);
        }
    }
};

} // namespace voxelization 