#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>

namespace VXZ {

// Cylinder voxelizer CPU implementation
class CylinderVoxelizerCPU : public VoxelizerCPU {
public:
    CylinderVoxelizerCPU(const Eigen::Vector3f& center,
                        const Eigen::Vector3f& axis,
                        float radius,
                        float height)
        : center_(center), axis_(axis.normalized()), radius_(radius), height_(height) {}
    
    void voxelize(VoxelGrid& grid) override;
    
private:
    Eigen::Vector3f center_;
    Eigen::Vector3f axis_;
    float radius_;
    float height_;
};

// Cylinder voxelizer GPU implementation
class CylinderVoxelizerGPU : public VoxelizerGPU {
public:
    CylinderVoxelizerGPU(const Eigen::Vector3f& center,
                        const Eigen::Vector3f& axis,
                        float radius,
                        float height)
        : center_(center), axis_(axis.normalized()), radius_(radius), height_(height) {}
    
    void voxelize(VoxelGrid& grid) override;
    
private:
    Eigen::Vector3f center_;
    Eigen::Vector3f axis_;
    float radius_;
    float height_;
};

// Factory class for CylinderVoxelizer
class CylinderVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(const Eigen::Vector3f& center,
                                               const Eigen::Vector3f& axis,
                                               float radius,
                                               float height,
                                               bool use_gpu = false) {
        if (use_gpu) {
            return std::make_unique<CylinderVoxelizerGPU>(center, axis, radius, height);
        } else {
            return std::make_unique<CylinderVoxelizerCPU>(center, axis, radius, height);
        }
    }
};

} // namespace VXZ