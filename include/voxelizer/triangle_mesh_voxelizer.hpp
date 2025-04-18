#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>

namespace VXZ {

// Triangle structure for mesh representation
struct Triangle {
    Eigen::Vector3f v0, v1, v2;
    Eigen::Vector3f normal;
    
    Triangle(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
        : v0(v0), v1(v1), v2(v2) {
        // Calculate normal
        Eigen::Vector3f edge1 = v1 - v0;
        Eigen::Vector3f edge2 = v2 - v0;
        normal = edge1.cross(edge2).normalized();
    }
};

// Triangle mesh voxelizer CPU implementation
class TriangleMeshVoxelizerCPU : public VoxelizerCPU {
public:
    TriangleMeshVoxelizerCPU(const std::vector<Triangle>& triangles)
        : triangles_(triangles) {}
    
    void voxelize_cpu(VoxelGrid& grid) override;
    
private:
    std::vector<Triangle> triangles_;
    
    // Helper functions
    bool ray_triangle_intersection(const Eigen::Vector3f& origin,
                                 const Eigen::Vector3f& direction,
                                 const Triangle& triangle,
                                 float& t) const;
};

// Triangle mesh voxelizer GPU implementation
class TriangleMeshVoxelizerGPU : public VoxelizerGPU {
public:
    TriangleMeshVoxelizerGPU(const std::vector<Triangle>& triangles)
        : triangles_(triangles) {}
    
    void voxelize_gpu(VoxelGrid& grid) override;
    
private:
    std::vector<Triangle> triangles_;
};

// Factory class for TriangleMeshVoxelizer
class TriangleMeshVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(const std::vector<Triangle>& triangles,
                                               bool use_gpu = false) {
        if (use_gpu) {
            return std::make_unique<TriangleMeshVoxelizerGPU>(triangles);
        } else {
            return std::make_unique<TriangleMeshVoxelizerCPU>(triangles);
        }
    }
};

} // namespace voxelization 