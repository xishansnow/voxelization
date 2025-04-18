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
    
    /**
     * @brief Voxelize the triangle mesh on CPU
     * 
     * This method implements triangle mesh voxelization using ray casting.
     * It determines which voxels are inside the mesh by casting rays and
     * counting intersections.
     * 
     * @param grid The voxel grid to fill
     */
    void voxelize(VoxelGrid& grid) override;
    
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
    
    /**
     * @brief Voxelize the triangle mesh on GPU
     * 
     * This method implements triangle mesh voxelization using parallel ray casting.
     * It determines which voxels are inside the mesh by casting rays and
     * counting intersections, utilizing GPU acceleration.
     * 
     * @param grid The voxel grid to fill
     */
    void voxelize(VoxelGrid& grid) override;
    
private:
    std::vector<Triangle> triangles_;
};



} // namespace VXZ