#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>

namespace VXZ {

// Face structure for polyhedron representation
struct Face {
    std::vector<Eigen::Vector3f> vertices;
    Eigen::Vector3f normal;
    
    Face(const std::vector<Eigen::Vector3f>& vertices)
        : vertices(vertices) {
        // Calculate normal using Newell's method
        normal = Eigen::Vector3f::Zero();
        for (size_t i = 0; i < vertices.size(); ++i) {
            const Eigen::Vector3f& current = vertices[i];
            const Eigen::Vector3f& next = vertices[(i + 1) % vertices.size()];
            normal.x() += (current.y() - next.y()) * (current.z() + next.z());
            normal.y() += (current.z() - next.z()) * (current.x() + next.x());
            normal.z() += (current.x() - next.x()) * (current.y() + next.y());
        }
        normal.normalize();
    }
};

// Polyhedron voxelizer CPU implementation
class PolyhedronVoxelizerCPU : public VoxelizerCPU {
public:
    PolyhedronVoxelizerCPU(const std::vector<Face>& faces)
        : faces_(faces) {}
    
    void voxelize_cpu(VoxelGrid& grid) override;
    
private:
    std::vector<Face> faces_;
    
    // Helper functions
    bool ray_face_intersection(const Eigen::Vector3f& origin,
                             const Eigen::Vector3f& direction,
                             const Face& face,
                             float& t) const;
};

// Polyhedron voxelizer GPU implementation
class PolyhedronVoxelizerGPU : public VoxelizerGPU {
public:
    PolyhedronVoxelizerGPU(const std::vector<Face>& faces)
        : faces_(faces) {}
    
    void voxelize_gpu(VoxelGrid& grid) override;
    
private:
    std::vector<Face> faces_;
};

// Factory class for PolyhedronVoxelizer
class PolyhedronVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(const std::vector<Face>& faces,
                                               bool use_gpu = false) {
        if (use_gpu) {
            return std::make_unique<PolyhedronVoxelizerGPU>(faces);
        } else {
            return std::make_unique<PolyhedronVoxelizerCPU>(faces);
        }
    }
};

} // namespace voxelization 