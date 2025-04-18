#pragma once

#include "../core/voxel_grid.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <functional>

namespace VXZ {

class VoxelizerKits {
public:
    // Box voxelization
    static VoxelGrid voxelize_box(const Eigen::Vector3f& center,
                                const Eigen::Vector3f& size,
                                float resolution,
                                const Eigen::Vector3f& min_bounds,
                                const Eigen::Vector3f& max_bounds);
    
    // Sphere voxelization
    static VoxelGrid voxelize_sphere(const Eigen::Vector3f& center,
                                   float radius,
                                   float resolution,
                                   const Eigen::Vector3f& min_bounds,
                                   const Eigen::Vector3f& max_bounds);
    
    // Corridor voxelization
    static VoxelGrid voxelize_corridor(const std::vector<Eigen::Vector3f>& waypoints,
                                     float width,
                                     float height,
                                     float resolution,
                                     const Eigen::Vector3f& min_bounds,
                                     const Eigen::Vector3f& max_bounds);
    
    // Mesh voxelization (using Marching Cubes)
    static VoxelGrid voxelize_mesh(const std::vector<Eigen::Vector3f>& vertices,
                                 const std::vector<Eigen::Vector3i>& faces,
                                 float resolution,
                                 const Eigen::Vector3f& min_bounds,
                                 const Eigen::Vector3f& max_bounds);
    
    // Cylinder voxelization
    static VoxelGrid voxelize_cylinder(const Eigen::Vector3f& center,
                              const Eigen::Vector3f& axis,
                              float radius,
                              float height,
                              float resolution,
                              const Eigen::Vector3f& min_bounds,
                              const Eigen::Vector3f& max_bounds);

    // Cone voxelization
    static VoxelGrid voxelize_cone(const Eigen::Vector3f& apex,
                          const Eigen::Vector3f& axis,
                          float radius,
                          float height,
                          float resolution,
                          const Eigen::Vector3f& min_bounds,
                          const Eigen::Vector3f& max_bounds);

    // Torus voxelization
    static VoxelGrid voxelize_torus(const Eigen::Vector3f& center,
                           const Eigen::Vector3f& axis,
                           float major_radius,
                           float minor_radius,
                           float resolution,
                           const Eigen::Vector3f& min_bounds,
                           const Eigen::Vector3f& max_bounds);

    // Capsule voxelization
    static VoxelGrid voxelize_capsule(const Eigen::Vector3f& start,
                             const Eigen::Vector3f& end,
                             float radius,
                             float resolution,
                             const Eigen::Vector3f& min_bounds,
                             const Eigen::Vector3f& max_bounds);

    // Point cloud voxelization
    static VoxelGrid voxelize_point_cloud(const std::vector<Eigen::Vector3f>& points,
                                        float resolution,
                                        const Eigen::Vector3f& min_bounds,
                                        const Eigen::Vector3f& max_bounds,
                                        float point_radius = 0.0f);

    // Implicit surface voxelization
    static VoxelGrid voxelize_implicit_surface(
        const std::function<float(const Eigen::Vector3f&)>& sdf,
        float resolution,
        const Eigen::Vector3f& min_bounds,
        const Eigen::Vector3f& max_bounds,
        float isovalue = 0.0f);

    // Signed distance field voxelization
    static VoxelGrid voxelize_sdf(const std::vector<float>& sdf_values,
                                const Eigen::Vector3i& dimensions,
                                float resolution,
                                const Eigen::Vector3f& min_bounds,
                                const Eigen::Vector3f& max_bounds,
                                float isovalue = 0.0f);

    // Marching cubes surface extraction
    static void extract_surface(const VoxelGrid& grid,
                              std::vector<Eigen::Vector3f>& vertices,
                              std::vector<Eigen::Vector3i>& faces,
                              float isovalue = 0.0f);

    // Line voxelization algorithms
    static VoxelGrid voxelize_line_rlv(const Eigen::Vector3f& start,
                                     const Eigen::Vector3f& end,
                                     float resolution,
                                     const Eigen::Vector3f& min_bounds,
                                     const Eigen::Vector3f& max_bounds);

    static VoxelGrid voxelize_line_slv(const Eigen::Vector3f& start,
                                     const Eigen::Vector3f& end,
                                     float resolution,
                                     const Eigen::Vector3f& min_bounds,
                                     const Eigen::Vector3f& max_bounds);

    static VoxelGrid voxelize_line_ilv(const Eigen::Vector3f& start,
                                     const Eigen::Vector3f& end,
                                     float resolution,
                                     const Eigen::Vector3f& min_bounds,
                                     const Eigen::Vector3f& max_bounds);

    static VoxelGrid voxelize_line_bresenham(const Eigen::Vector3f& start,
                                           const Eigen::Vector3f& end,
                                           float resolution,
                                           const Eigen::Vector3f& min_bounds,
                                           const Eigen::Vector3f& max_bounds);

private:
    // CPU implementations
    static void voxelize_box_cpu(VoxelGrid& grid,
                               const Eigen::Vector3f& center,
                               const Eigen::Vector3f& size);
    
    static void voxelize_sphere_cpu(VoxelGrid& grid,
                                  const Eigen::Vector3f& center,
                                  float radius);
    
    static void voxelize_corridor_cpu(VoxelGrid& grid,
                                    const std::vector<Eigen::Vector3f>& waypoints,
                                    float width,
                                    float height);
    
    static void voxelize_mesh_cpu(VoxelGrid& grid,
                                const std::vector<Eigen::Vector3f>& vertices,
                                const std::vector<Eigen::Vector3i>& faces);

    // CPU implementations for new primitives
    static void voxelize_cylinder_cpu(VoxelGrid& grid,
                             const Eigen::Vector3f& center,
                             const Eigen::Vector3f& axis,
                             float radius,
                             float height);

    static void voxelize_cone_cpu(VoxelGrid& grid,
                         const Eigen::Vector3f& apex,
                         const Eigen::Vector3f& axis,
                         float radius,
                         float height);

    static void voxelize_torus_cpu(VoxelGrid& grid,
                          const Eigen::Vector3f& center,
                          const Eigen::Vector3f& axis,
                          float major_radius,
                          float minor_radius);

    static void voxelize_capsule_cpu(VoxelGrid& grid,
                            const Eigen::Vector3f& start,
                            const Eigen::Vector3f& end,
                            float radius);

    // CPU implementations for surface models
    static void voxelize_point_cloud_cpu(VoxelGrid& grid,
                                       const std::vector<Eigen::Vector3f>& points,
                                       float point_radius);

    static void voxelize_implicit_surface_cpu(
        VoxelGrid& grid,
        const std::function<float(const Eigen::Vector3f&)>& sdf,
        float isovalue);

    static void voxelize_sdf_cpu(VoxelGrid& grid,
                               const std::vector<float>& sdf_values,
                               const Eigen::Vector3i& dimensions,
                               float isovalue);

    static void extract_surface_cpu(const VoxelGrid& grid,
                                 std::vector<Eigen::Vector3f>& vertices,
                                 std::vector<Eigen::Vector3i>& faces,
                                 float isovalue);

    // CPU implementations for line algorithms
    static void voxelize_line_rlv_cpu(VoxelGrid& grid,
                                    const Eigen::Vector3f& start,
                                    const Eigen::Vector3f& end);

    static void voxelize_line_slv_cpu(VoxelGrid& grid,
                                    const Eigen::Vector3f& start,
                                    const Eigen::Vector3f& end);

    static void voxelize_line_ilv_cpu(VoxelGrid& grid,
                                    const Eigen::Vector3f& start,
                                    const Eigen::Vector3f& end);

    static void voxelize_line_bresenham_cpu(VoxelGrid& grid,
                                          const Eigen::Vector3f& start,
                                          const Eigen::Vector3f& end);
};

} // namespace VXZ