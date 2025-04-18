#include "voxelizer/triangle_mesh_voxelizer.hpp"

namespace VXZ {

void TriangleMeshVoxelizerCPU::voxelize(VoxelGrid& grid) {
    // Calculate mesh bounding box
    Eigen::Vector3f min = triangles_[0].v0;
    Eigen::Vector3f max = triangles_[0].v0;
    
    for (const auto& triangle : triangles_) {
        min = min.cwiseMin(triangle.v0).cwiseMin(triangle.v1).cwiseMin(triangle.v2);
        max = max.cwiseMax(triangle.v0).cwiseMax(triangle.v1).cwiseMax(triangle.v2);
    }
    
    // Convert to grid coordinates
    Eigen::Vector3i grid_min = grid.world_to_grid(min);
    Eigen::Vector3i grid_max = grid.world_to_grid(max);
    
    // Clamp to grid bounds
    grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
    grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
    
    // Process each voxel
    for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
        for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
            for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                
                // Cast ray in +x direction
                Eigen::Vector3f ray_origin = world_pos;
                Eigen::Vector3f ray_direction(1.0f, 0.0f, 0.0f);
                
                int intersections = 0;
                for (const auto& triangle : triangles_) {
                    float t;
                    if (ray_triangle_intersection(ray_origin, ray_direction, triangle, t)) {
                        if (t >= 0) {
                            intersections++;
                        }
                    }
                }
                
                // If odd number of intersections, point is inside mesh
                if (intersections % 2 == 1) {
                    grid.set(Eigen::Vector3i(x, y, z), true);
                }
            }
        }
    }
}

bool TriangleMeshVoxelizerCPU::ray_triangle_intersection(
    const Eigen::Vector3f& origin,
    const Eigen::Vector3f& direction,
    const Triangle& triangle,
    float& t) const {
    
    const float EPSILON = 1e-6f;
    
    Eigen::Vector3f edge1 = triangle.v1 - triangle.v0;
    Eigen::Vector3f edge2 = triangle.v2 - triangle.v0;
    
    Eigen::Vector3f h = direction.cross(edge2);
    float a = edge1.dot(h);
    
    if (std::abs(a) < EPSILON) {
        return false; // Ray is parallel to triangle
    }
    
    float f = 1.0f / a;
    Eigen::Vector3f s = origin - triangle.v0;
    float u = f * s.dot(h);
    
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    
    Eigen::Vector3f q = s.cross(edge1);
    float v = f * direction.dot(q);
    
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    
    t = f * edge2.dot(q);
    return t > EPSILON;
}

void TriangleMeshVoxelizerGPU::voxelize(VoxelGrid& grid) {

} 
}// namespace VXZ 