#include "voxelizer/voxelizer.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace VXZ {

// Box voxelization
VoxelGrid VoxelizerKits::voxelize_box(const Eigen::Vector3f& center,
                                const Eigen::Vector3f& size,
                                float resolution,
                                const Eigen::Vector3f& min_bounds,
                                const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_box_cpu(grid, center, size);
    return grid;
}

// Sphere voxelization
VoxelGrid VoxelizerKits::voxelize_sphere(const Eigen::Vector3f& center,
                                   float radius,
                                   float resolution,
                                   const Eigen::Vector3f& min_bounds,
                                   const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_sphere_cpu(grid, center, radius);
    return grid;
}

// Corridor voxelization
VoxelGrid VoxelizerKits::voxelize_corridor(const std::vector<Eigen::Vector3f>& waypoints,
                                     float width,
                                     float height,
                                     float resolution,
                                     const Eigen::Vector3f& min_bounds,
                                     const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_corridor_cpu(grid, waypoints, width, height);
    return grid;
}

// Mesh voxelization
VoxelGrid VoxelizerKits::voxelize_mesh(const std::vector<Eigen::Vector3f>& vertices,
                                 const std::vector<Eigen::Vector3i>& faces,
                                 float resolution,
                                 const Eigen::Vector3f& min_bounds,
                                 const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_mesh_cpu(grid, vertices, faces);
    return grid;
}

// Cylinder voxelization
VoxelGrid VoxelizerKits::voxelize_cylinder(const Eigen::Vector3f& center,
                                     const Eigen::Vector3f& axis,
                                     float radius,
                                     float height,
                                     float resolution,
                                     const Eigen::Vector3f& min_bounds,
                                     const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_cylinder_cpu(grid, center, axis, radius, height);
    return grid;
}

// Cone voxelization
VoxelGrid VoxelizerKits::voxelize_cone(const Eigen::Vector3f& apex,
                                 const Eigen::Vector3f& axis,
                                 float radius,
                                 float height,
                                 float resolution,
                                 const Eigen::Vector3f& min_bounds,
                                 const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_cone_cpu(grid, apex, axis, radius, height);
    return grid;
}

// Torus voxelization
VoxelGrid VoxelizerKits::voxelize_torus(const Eigen::Vector3f& center,
                                  const Eigen::Vector3f& axis,
                                  float major_radius,
                                  float minor_radius,
                                  float resolution,
                                  const Eigen::Vector3f& min_bounds,
                                  const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_torus_cpu(grid, center, axis, major_radius, minor_radius);
    return grid;
}

// Capsule voxelization
VoxelGrid VoxelizerKits::voxelize_capsule(const Eigen::Vector3f& start,
                                    const Eigen::Vector3f& end,
                                    float radius,
                                    float resolution,
                                    const Eigen::Vector3f& min_bounds,
                                    const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_capsule_cpu(grid, start, end, radius);
    return grid;
}

// Point cloud voxelization
VoxelGrid VoxelizerKits::voxelize_point_cloud(const std::vector<Eigen::Vector3f>& points,
                                        float resolution,
                                        const Eigen::Vector3f& min_bounds,
                                        const Eigen::Vector3f& max_bounds,
                                        float point_radius) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_point_cloud_cpu(grid, points, point_radius);
    return grid;
}

// Implicit surface voxelization
VoxelGrid VoxelizerKits::voxelize_implicit_surface(
    const std::function<float(const Eigen::Vector3f&)>& sdf,
    float resolution,
    const Eigen::Vector3f& min_bounds,
    const Eigen::Vector3f& max_bounds,
    float isovalue) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_implicit_surface_cpu(grid, sdf, isovalue);
    return grid;
}

// Signed distance field voxelization
VoxelGrid VoxelizerKits::voxelize_sdf(const std::vector<float>& sdf_values,
                                const Eigen::Vector3i& dimensions,
                                float resolution,
                                const Eigen::Vector3f& min_bounds,
                                const Eigen::Vector3f& max_bounds,
                                float isovalue) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_sdf_cpu(grid, sdf_values, dimensions, isovalue);
    return grid;
}

// Marching cubes surface extraction
void VoxelizerKits::extract_surface(const VoxelGrid& grid,
                              std::vector<Eigen::Vector3f>& vertices,
                              std::vector<Eigen::Vector3i>& faces,
                              float isovalue) {
    extract_surface_cpu(grid, vertices, faces, isovalue);
}

// CPU implementations
void VoxelizerKits::voxelize_box_cpu(VoxelGrid& grid,
                               const Eigen::Vector3f& center,
                               const Eigen::Vector3f& size) {
    Eigen::Vector3f half_size = size * 0.5f;
    Eigen::Vector3f min = center - half_size;
    Eigen::Vector3f max = center + half_size;
    
    // Convert bounds to grid coordinates
    Eigen::Vector3i grid_min = grid.world_to_grid(min);
    Eigen::Vector3i grid_max = grid.world_to_grid(max);
    
    // Clamp to grid bounds
    grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
    grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
    
    // Fill the box
    for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
        for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
            for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                grid.set(Eigen::Vector3i(x, y, z), true);
            }
        }
    }
}

void VoxelizerKits::voxelize_sphere_cpu(VoxelGrid& grid,
                                  const Eigen::Vector3f& center,
                                  float radius) {
    float radius_squared = radius * radius;
    
    // Convert center to grid coordinates
    Eigen::Vector3i grid_center = grid.world_to_grid(center);
    
    // Calculate bounding box in grid coordinates
    int radius_in_voxels = std::ceil(radius / grid.resolution());
    Eigen::Vector3i grid_min = grid_center - Eigen::Vector3i::Constant(radius_in_voxels);
    Eigen::Vector3i grid_max = grid_center + Eigen::Vector3i::Constant(radius_in_voxels);
    
    // Clamp to grid bounds
    grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
    grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
    
    // Fill the sphere
    for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
        for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
            for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                float dist_squared = (world_pos - center).squaredNorm();
                if (dist_squared <= radius_squared) {
                    grid.set(Eigen::Vector3i(x, y, z), true);
                }
            }
        }
    }
}

void VoxelizerKits::voxelize_corridor_cpu(VoxelGrid& grid,
                                    const std::vector<Eigen::Vector3f>& waypoints,
                                    float width,
                                    float height) {
    if (waypoints.size() < 2) return;
    
    float half_width = width * 0.5f;
    float half_height = height * 0.5f;
    
    // Process each segment
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const Eigen::Vector3f& p1 = waypoints[i];
        const Eigen::Vector3f& p2 = waypoints[i + 1];
        
        // Calculate segment direction and length
        Eigen::Vector3f dir = p2 - p1;
        float length = dir.norm();
        dir.normalize();
        
        // Calculate perpendicular vectors
        Eigen::Vector3f perp1 = dir.cross(Eigen::Vector3f::UnitY()).normalized();
        Eigen::Vector3f perp2 = dir.cross(perp1).normalized();
        
        // Calculate bounding box
        Eigen::Vector3f min = p1 - Eigen::Vector3f(half_width, half_height, half_width);
        Eigen::Vector3f max = p2 + Eigen::Vector3f(half_width, half_height, half_width);
        
        // Convert to grid coordinates
        Eigen::Vector3i grid_min = grid.world_to_grid(min);
        Eigen::Vector3i grid_max = grid.world_to_grid(max);
        
        // Clamp to grid bounds
        grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
        grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
        
        // Fill the corridor segment
        for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
            for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
                for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                    Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                    
                    // Project point onto segment
                    float t = (world_pos - p1).dot(dir);
                    t = std::max(0.0f, std::min(t, length));
                    Eigen::Vector3f proj = p1 + t * dir;
                    
                    // Check distance to segment
                    float dist = (world_pos - proj).norm();
                    if (dist <= half_width && std::abs(world_pos.y() - proj.y()) <= half_height) {
                        grid.set(Eigen::Vector3i(x, y, z), true);
                    }
                }
            }
        }
    }
}

void VoxelizerKits::voxelize_mesh_cpu(VoxelGrid& grid,
                                const std::vector<Eigen::Vector3f>& vertices,
                                const std::vector<Eigen::Vector3i>& faces) {
    // Calculate mesh bounding box
    Eigen::Vector3f min = vertices[0];
    Eigen::Vector3f max = vertices[0];
    for (const auto& v : vertices) {
        min = min.cwiseMin(v);
        max = max.cwiseMax(v);
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
                
                // Check if point is inside any triangle
                bool inside = false;
                for (const auto& face : faces) {
                    const Eigen::Vector3f& v0 = vertices[face[0]];
                    const Eigen::Vector3f& v1 = vertices[face[1]];
                    const Eigen::Vector3f& v2 = vertices[face[2]];
                    
                    // Calculate barycentric coordinates
                    Eigen::Vector3f v0v1 = v1 - v0;
                    Eigen::Vector3f v0v2 = v2 - v0;
                    Eigen::Vector3f v0p = world_pos - v0;
                    
                    float d00 = v0v1.dot(v0v1);
                    float d01 = v0v1.dot(v0v2);
                    float d11 = v0v2.dot(v0v2);
                    float d20 = v0p.dot(v0v1);
                    float d21 = v0p.dot(v0v2);
                    
                    float denom = d00 * d11 - d01 * d01;
                    float v = (d11 * d20 - d01 * d21) / denom;
                    float w = (d00 * d21 - d01 * d20) / denom;
                    float u = 1.0f - v - w;
                    
                    if (u >= 0 && v >= 0 && w >= 0) {
                        inside = true;
                        break;
                    }
                }
                
                if (inside) {
                    grid.set(Eigen::Vector3i(x, y, z), true);
                }
            }
        }
    }
}

// CPU implementations for new primitives
void VoxelizerKits::voxelize_cylinder_cpu(VoxelGrid& grid,
                                    const Eigen::Vector3f& center,
                                    const Eigen::Vector3f& axis,
                                    float radius,
                                    float height) {
    float radius_squared = radius * radius;
    Eigen::Vector3f axis_normalized = axis.normalized();
    Eigen::Vector3f half_height = axis_normalized * (height * 0.5f);
    
    // Calculate bounding box
    Eigen::Vector3f min = center - half_height - Eigen::Vector3f::Constant(radius);
    Eigen::Vector3f max = center + half_height + Eigen::Vector3f::Constant(radius);
    
    // Convert to grid coordinates
    Eigen::Vector3i grid_min = grid.world_to_grid(min);
    Eigen::Vector3i grid_max = grid.world_to_grid(max);
    
    // Clamp to grid bounds
    grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
    grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
    
    // Fill the cylinder
    for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
        for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
            for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                
                // Project point onto cylinder axis
                Eigen::Vector3f to_point = world_pos - center;
                float projection = to_point.dot(axis_normalized);
                
                // Check if point is within height bounds
                if (std::abs(projection) <= height * 0.5f) {
                    // Calculate distance from axis
                    Eigen::Vector3f point_on_axis = center + axis_normalized * projection;
                    float dist_squared = (world_pos - point_on_axis).squaredNorm();
                    
                    if (dist_squared <= radius_squared) {
                        grid.set(Eigen::Vector3i(x, y, z), true);
                    }
                }
            }
        }
    }
}

void VoxelizerKits::voxelize_cone_cpu(VoxelGrid& grid,
                                const Eigen::Vector3f& apex,
                                const Eigen::Vector3f& axis,
                                float radius,
                                float height) {
    Eigen::Vector3f axis_normalized = axis.normalized();
    Eigen::Vector3f base_center = apex + axis_normalized * height;
    
    // Calculate bounding box
    Eigen::Vector3f min = apex - Eigen::Vector3f::Constant(radius);
    Eigen::Vector3f max = base_center + Eigen::Vector3f::Constant(radius);
    
    // Convert to grid coordinates
    Eigen::Vector3i grid_min = grid.world_to_grid(min);
    Eigen::Vector3i grid_max = grid.world_to_grid(max);
    
    // Clamp to grid bounds
    grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
    grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
    
    // Fill the cone
    for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
        for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
            for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                
                // Project point onto cone axis
                Eigen::Vector3f to_point = world_pos - apex;
                float projection = to_point.dot(axis_normalized);
                
                // Check if point is within height bounds
                if (projection >= 0 && projection <= height) {
                    // Calculate local radius at this height
                    float local_radius = radius * (1.0f - projection / height);
                    float local_radius_squared = local_radius * local_radius;
                    
                    // Calculate distance from axis
                    Eigen::Vector3f point_on_axis = apex + axis_normalized * projection;
                    float dist_squared = (world_pos - point_on_axis).squaredNorm();
                    
                    if (dist_squared <= local_radius_squared) {
                        grid.set(Eigen::Vector3i(x, y, z), true);
                    }
                }
            }
        }
    }
}

void VoxelizerKits::voxelize_torus_cpu(VoxelGrid& grid,
                                 const Eigen::Vector3f& center,
                                 const Eigen::Vector3f& axis,
                                 float major_radius,
                                 float minor_radius) {
    float minor_radius_squared = minor_radius * minor_radius;
    Eigen::Vector3f axis_normalized = axis.normalized();
    
    // Calculate bounding box
    float total_radius = major_radius + minor_radius;
    Eigen::Vector3f min = center - Eigen::Vector3f::Constant(total_radius);
    Eigen::Vector3f max = center + Eigen::Vector3f::Constant(total_radius);
    
    // Convert to grid coordinates
    Eigen::Vector3i grid_min = grid.world_to_grid(min);
    Eigen::Vector3i grid_max = grid.world_to_grid(max);
    
    // Clamp to grid bounds
    grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
    grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
    
    // Fill the torus
    for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
        for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
            for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                
                // Project point onto torus plane
                Eigen::Vector3f to_point = world_pos - center;
                float projection = to_point.dot(axis_normalized);
                Eigen::Vector3f point_on_plane = world_pos - axis_normalized * projection;
                
                // Calculate distance from major circle
                float dist_to_major = (point_on_plane - center).norm();
                float dist_to_major_squared = (dist_to_major - major_radius) * (dist_to_major - major_radius);
                
                // Check if point is within minor radius
                if (dist_to_major_squared + projection * projection <= minor_radius_squared) {
                    grid.set(Eigen::Vector3i(x, y, z), true);
                }
            }
        }
    }
}

void VoxelizerKits::voxelize_capsule_cpu(VoxelGrid& grid,
                                   const Eigen::Vector3f& start,
                                   const Eigen::Vector3f& end,
                                   float radius) {
    float radius_squared = radius * radius;
    Eigen::Vector3f dir = end - start;
    float length = dir.norm();
    dir.normalize();
    
    // Calculate bounding box
    Eigen::Vector3f min = start.cwiseMin(end) - Eigen::Vector3f::Constant(radius);
    Eigen::Vector3f max = start.cwiseMax(end) + Eigen::Vector3f::Constant(radius);
    
    // Convert to grid coordinates
    Eigen::Vector3i grid_min = grid.world_to_grid(min);
    Eigen::Vector3i grid_max = grid.world_to_grid(max);
    
    // Clamp to grid bounds
    grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
    grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
    
    // Fill the capsule
    for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
        for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
            for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                
                // Project point onto capsule axis
                float t = (world_pos - start).dot(dir);
                t = std::max(0.0f, std::min(t, length));
                Eigen::Vector3f proj = start + t * dir;
                
                // Check distance to axis
                float dist_squared = (world_pos - proj).squaredNorm();
                if (dist_squared <= radius_squared) {
                    grid.set(Eigen::Vector3i(x, y, z), true);
                }
            }
        }
    }
}

// CPU implementations for surface models
void VoxelizerKits::voxelize_point_cloud_cpu(VoxelGrid& grid,
                                       const std::vector<Eigen::Vector3f>& points,
                                       float point_radius) {
    if (point_radius <= 0.0f) {
        // Simple point voxelization
        for (const auto& point : points) {
            Eigen::Vector3i grid_pos = grid.world_to_grid(point);
            if (grid.is_valid_position(grid_pos)) {
                grid.set(grid_pos, true);
            }
        }
    } else {
        // Point with radius voxelization
        float radius_squared = point_radius * point_radius;
        for (const auto& point : points) {
            // Calculate bounding box
            Eigen::Vector3f min = point - Eigen::Vector3f::Constant(point_radius);
            Eigen::Vector3f max = point + Eigen::Vector3f::Constant(point_radius);
            
            // Convert to grid coordinates
            Eigen::Vector3i grid_min = grid.world_to_grid(min);
            Eigen::Vector3i grid_max = grid.world_to_grid(max);
            
            // Clamp to grid bounds
            grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
            grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
            
            // Fill the sphere around the point
            for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
                for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
                    for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                        Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                        float dist_squared = (world_pos - point).squaredNorm();
                        if (dist_squared <= radius_squared) {
                            grid.set(Eigen::Vector3i(x, y, z), true);
                        }
                    }
                }
            }
        }
    }
}

void VoxelizerKits::voxelize_implicit_surface_cpu(
    VoxelGrid& grid,
    const std::function<float(const Eigen::Vector3f&)>& sdf,
    float isovalue) {
    // Process each voxel
    for (int x = 0; x < grid.dimensions().x(); ++x) {
        for (int y = 0; y < grid.dimensions().y(); ++y) {
            for (int z = 0; z < grid.dimensions().z(); ++z) {
                Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                float value = sdf(world_pos);
                grid.set(Eigen::Vector3i(x, y, z), value <= isovalue);
            }
        }
    }
}

void VoxelizerKits::voxelize_sdf_cpu(VoxelGrid& grid,
                               const std::vector<float>& sdf_values,
                               const Eigen::Vector3i& dimensions,
                               float isovalue) {
    // Check if dimensions match
    if (dimensions != grid.dimensions()) {
        throw std::runtime_error("SDF dimensions do not match grid dimensions");
    }
    
    // Process each voxel
    for (int x = 0; x < dimensions.x(); ++x) {
        for (int y = 0; y < dimensions.y(); ++y) {
            for (int z = 0; z < dimensions.z(); ++z) {
                int index = x + y * dimensions.x() + z * dimensions.x() * dimensions.y();
                float value = sdf_values[index];
                grid.set(Eigen::Vector3i(x, y, z), value <= isovalue);
            }
        }
    }
}

// Marching cubes tables
namespace {
    // Edge table for marching cubes
    const int edgeTable[256] = {
        0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
        0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
        0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
        0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
        0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
        0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
        0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
        0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
        0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
        0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
        0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
        0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
        0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
        0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
        0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
        0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0
    };

    // Triangle table for marching cubes
    const int triTable[256][16] = {
        {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        // ... (完整的三维查找表)
    };
}

void VoxelizerKits::extract_surface_cpu(const VoxelGrid& grid,
                                  std::vector<Eigen::Vector3f>& vertices,
                                  std::vector<Eigen::Vector3i>& faces,
                                  float isovalue) {
    vertices.clear();
    faces.clear();
    
    // Create a map to store vertex indices for each edge
    std::unordered_map<uint64_t, int> edgeToVertex;
    
    // Process each voxel
    for (int x = 0; x < grid.dimensions().x() - 1; ++x) {
        for (int y = 0; y < grid.dimensions().y() - 1; ++y) {
            for (int z = 0; z < grid.dimensions().z() - 1; ++z) {
                // Get the 8 corners of the voxel
                Eigen::Vector3i corners[8] = {
                    Eigen::Vector3i(x, y, z),
                    Eigen::Vector3i(x + 1, y, z),
                    Eigen::Vector3i(x + 1, y + 1, z),
                    Eigen::Vector3i(x, y + 1, z),
                    Eigen::Vector3i(x, y, z + 1),
                    Eigen::Vector3i(x + 1, y, z + 1),
                    Eigen::Vector3i(x + 1, y + 1, z + 1),
                    Eigen::Vector3i(x, y + 1, z + 1)
                };
                
                // Calculate the case index
                int caseIndex = 0;
                for (int i = 0; i < 8; ++i) {
                    if (grid.get(corners[i])) {
                        caseIndex |= (1 << i);
                    }
                }
                
                // Skip empty or full voxels
                if (caseIndex == 0 || caseIndex == 255) {
                    continue;
                }
                
                // Get the edges for this case
                int edges = edgeTable[caseIndex];
                
                // Process each edge
                for (int i = 0; i < 12; ++i) {
                    if (edges & (1 << i)) {
                        // Get the two vertices of this edge
                        int v1 = i;
                        int v2 = (i + 1) % 8;
                        
                        // Create a unique key for this edge
                        uint64_t key = (static_cast<uint64_t>(corners[v1].x()) << 48) |
                                     (static_cast<uint64_t>(corners[v1].y()) << 32) |
                                     (static_cast<uint64_t>(corners[v1].z()) << 16) |
                                     (static_cast<uint64_t>(corners[v2].x()) << 12) |
                                     (static_cast<uint64_t>(corners[v2].y()) << 8) |
                                     (static_cast<uint64_t>(corners[v2].z()) << 4);
                        
                        // Check if we already have a vertex for this edge
                        auto it = edgeToVertex.find(key);
                        if (it == edgeToVertex.end()) {
                            // Create a new vertex
                            Eigen::Vector3f p1 = grid.grid_to_world(corners[v1]);
                            Eigen::Vector3f p2 = grid.grid_to_world(corners[v2]);
                            Eigen::Vector3f vertex = (p1 + p2) * 0.5f;
                            
                            int vertexIndex = vertices.size();
                            vertices.push_back(vertex);
                            edgeToVertex[key] = vertexIndex;
                        }
                    }
                }
                
                // Create triangles for this case
                const int* triangles = triTable[caseIndex];
                for (int i = 0; triangles[i] != -1; i += 3) {
                    faces.push_back(Eigen::Vector3i(
                        edgeToVertex[triangles[i]],
                        edgeToVertex[triangles[i + 1]],
                        edgeToVertex[triangles[i + 2]]
                    ));
                }
            }
        }
    }
}

VoxelGrid VoxelizerKits::voxelize_line_rlv(const Eigen::Vector3f& start,
                                     const Eigen::Vector3f& end,
                                     float resolution,
                                     const Eigen::Vector3f& min_bounds,
                                     const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_line_rlv_cpu(grid, start, end);
    return grid;
}

VoxelGrid VoxelizerKits::voxelize_line_slv(const Eigen::Vector3f& start,
                                     const Eigen::Vector3f& end,
                                     float resolution,
                                     const Eigen::Vector3f& min_bounds,
                                     const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_line_slv_cpu(grid, start, end);
    return grid;
}

VoxelGrid VoxelizerKits::voxelize_line_ilv(const Eigen::Vector3f& start,
                                     const Eigen::Vector3f& end,
                                     float resolution,
                                     const Eigen::Vector3f& min_bounds,
                                     const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_line_ilv_cpu(grid, start, end);
    return grid;
}

VoxelGrid VoxelizerKits::voxelize_line_bresenham(const Eigen::Vector3f& start,
                                           const Eigen::Vector3f& end,
                                           float resolution,
                                           const Eigen::Vector3f& min_bounds,
                                           const Eigen::Vector3f& max_bounds) {
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    voxelize_line_bresenham_cpu(grid, start, end);
    return grid;
}

// RLV (Real Line Voxelisation) CPU implementation
void VoxelizerKits::voxelize_line_rlv_cpu(VoxelGrid& grid,
                                    const Eigen::Vector3f& start,
                                    const Eigen::Vector3f& end) {
    Eigen::Vector3f direction = end - start;
    float length = direction.norm();
    direction.normalize();

    // Calculate step size based on resolution
    float step_size = grid.resolution() * 0.5f;
    int num_steps = static_cast<int>(length / step_size) + 1;

    for (int i = 0; i <= num_steps; ++i) {
        float t = static_cast<float>(i) / num_steps;
        Eigen::Vector3f point = start + t * direction;
        grid.set(grid.world_to_grid(point), true);
    }
}

// SLV (Supercover Line Voxelisation) CPU implementation
void VoxelizerKits::voxelize_line_slv_cpu(VoxelGrid& grid,
                                    const Eigen::Vector3f& start,
                                    const Eigen::Vector3f& end) {
    Eigen::Vector3f direction = end - start;
    float length = direction.norm();
    direction.normalize();

    // Calculate step size based on resolution
    float step_size = grid.resolution() * 0.5f;
    int num_steps = static_cast<int>(length / step_size) + 1;

    for (int i = 0; i <= num_steps; ++i) {
        float t = static_cast<float>(i) / num_steps;
        Eigen::Vector3f point = start + t * direction;
        
        // Set the voxel containing the point
        grid.set(grid.world_to_grid(point), true);
        
        // Set neighboring voxels to ensure full coverage
        Eigen::Vector3i grid_pos = grid.world_to_grid(point);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    Eigen::Vector3i neighbor = grid_pos + Eigen::Vector3i(dx, dy, dz);
                    if (grid.is_inside_grid(neighbor)) {
                        grid.set(neighbor, true);
                    }
                }
            }
        }
    }
}

// ILV (Integer-only Line Voxelisation) CPU implementation
void VoxelizerKits::voxelize_line_ilv_cpu(VoxelGrid& grid,
                                    const Eigen::Vector3f& start,
                                    const Eigen::Vector3f& end) {
    Eigen::Vector3i start_grid = grid.world_to_grid(start);
    Eigen::Vector3i end_grid = grid.world_to_grid(end);
    
    int dx = std::abs(end_grid.x() - start_grid.x());
    int dy = std::abs(end_grid.y() - start_grid.y());
    int dz = std::abs(end_grid.z() - start_grid.z());
    
    int step_x = (start_grid.x() < end_grid.x()) ? 1 : -1;
    int step_y = (start_grid.y() < end_grid.y()) ? 1 : -1;
    int step_z = (start_grid.z() < end_grid.z()) ? 1 : -1;
    
    int x = start_grid.x();
    int y = start_grid.y();
    int z = start_grid.z();
    
    if (dx >= dy && dx >= dz) {
        int err_1 = 2 * dy - dx;
        int err_2 = 2 * dz - dx;
        for (int i = 0; i <= dx; ++i) {
            grid.set(Eigen::Vector3i(x, y, z), true);
            if (err_1 > 0) {
                y += step_y;
                err_1 -= 2 * dx;
            }
            if (err_2 > 0) {
                z += step_z;
                err_2 -= 2 * dx;
            }
            err_1 += 2 * dy;
            err_2 += 2 * dz;
            x += step_x;
        }
    } else if (dy >= dx && dy >= dz) {
        int err_1 = 2 * dx - dy;
        int err_2 = 2 * dz - dy;
        for (int i = 0; i <= dy; ++i) {
            grid.set(Eigen::Vector3i(x, y, z), true);
            if (err_1 > 0) {
                x += step_x;
                err_1 -= 2 * dy;
            }
            if (err_2 > 0) {
                z += step_z;
                err_2 -= 2 * dy;
            }
            err_1 += 2 * dx;
            err_2 += 2 * dz;
            y += step_y;
        }
    } else {
        int err_1 = 2 * dx - dz;
        int err_2 = 2 * dy - dz;
        for (int i = 0; i <= dz; ++i) {
            grid.set(Eigen::Vector3i(x, y, z), true);
            if (err_1 > 0) {
                x += step_x;
                err_1 -= 2 * dz;
            }
            if (err_2 > 0) {
                y += step_y;
                err_2 -= 2 * dz;
            }
            err_1 += 2 * dx;
            err_2 += 2 * dy;
            z += step_z;
        }
    }
}

// 3D Bresenham's line algorithm CPU implementation
void VoxelizerKits::voxelize_line_bresenham_cpu(VoxelGrid& grid,
                                          const Eigen::Vector3f& start,
                                          const Eigen::Vector3f& end) {
    Eigen::Vector3i start_grid = grid.world_to_grid(start);
    Eigen::Vector3i end_grid = grid.world_to_grid(end);
    
    int dx = std::abs(end_grid.x() - start_grid.x());
    int dy = std::abs(end_grid.y() - start_grid.y());
    int dz = std::abs(end_grid.z() - start_grid.z());
    
    int step_x = (start_grid.x() < end_grid.x()) ? 1 : -1;
    int step_y = (start_grid.y() < end_grid.y()) ? 1 : -1;
    int step_z = (start_grid.z() < end_grid.z()) ? 1 : -1;
    
    int x = start_grid.x();
    int y = start_grid.y();
    int z = start_grid.z();
    
    if (dx >= dy && dx >= dz) {
        int err_1 = 2 * dy - dx;
        int err_2 = 2 * dz - dx;
        for (int i = 0; i <= dx; ++i) {
            grid.set(Eigen::Vector3i(x, y, z), true);
            if (err_1 > 0) {
                y += step_y;
                err_1 -= 2 * dx;
            }
            if (err_2 > 0) {
                z += step_z;
                err_2 -= 2 * dx;
            }
            err_1 += 2 * dy;
            err_2 += 2 * dz;
            x += step_x;
        }
    } else if (dy >= dx && dy >= dz) {
        int err_1 = 2 * dx - dy;
        int err_2 = 2 * dz - dy;
        for (int i = 0; i <= dy; ++i) {
            grid.set(Eigen::Vector3i(x, y, z), true);
            if (err_1 > 0) {
                x += step_x;
                err_1 -= 2 * dy;
            }
            if (err_2 > 0) {
                z += step_z;
                err_2 -= 2 * dy;
            }
            err_1 += 2 * dx;
            err_2 += 2 * dz;
            y += step_y;
        }
    } else {
        int err_1 = 2 * dx - dz;
        int err_2 = 2 * dy - dz;
        for (int i = 0; i <= dz; ++i) {
            grid.set(Eigen::Vector3i(x, y, z), true);
            if (err_1 > 0) {
                x += step_x;
                err_1 -= 2 * dz;
            }
            if (err_2 > 0) {
                y += step_y;
                err_2 -= 2 * dz;
            }
            err_1 += 2 * dx;
            err_2 += 2 * dy;
            z += step_z;
        }
    }
}

} // namespace VXZ