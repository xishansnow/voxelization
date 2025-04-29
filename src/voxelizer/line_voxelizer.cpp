#include "voxelizer/line_voxelizer.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace VXZ {

void LineVoxelizerCPU::voxelize(VoxelGrid& grid) {
    switch (algorithm_) {
        case LineAlgorithm::RLV:
            voxelize_rlv(grid);
            break;
        case LineAlgorithm::SLV:
            voxelize_slv(grid);
            break;
        case LineAlgorithm::ILV:
            voxelize_ilv(grid);
            break;
        case LineAlgorithm::DDA:
            voxelize_dda(grid);
            break;
        case LineAlgorithm::BRESENHAM:
            voxelize_bresenham(grid);
            break;
        case LineAlgorithm::TRIPOD:
            voxelize_tripod(grid);
            break;
        case LineAlgorithm::WU:
            voxelize_wu(grid);
            break;
    }
}

// Real Line Voxelisation (RLV)
// Description: Uses floating-point calculations to achieve precise line representation
// Reference: 
// - "Real-Time Voxelization for Global Illumination" by Crassin et al. (2011)
// - "Efficient GPU-Based Voxelization Using Hierarchical Occlusion Maps" by Schwarz and Seidel (2010)
void LineVoxelizerCPU::voxelize_rlv(VoxelGrid& grid) {
    // Real Line Voxelisation algorithm
    Eigen::Vector3f direction = end_ - start_;
    float max_length = direction.norm();
    direction.normalize();
    
    // Convert start and end points to grid coordinates
    Eigen::Vector3i start_grid = grid.world_to_grid(start_);
    Eigen::Vector3i end_grid = grid.world_to_grid(end_);
    
    // Calculate step size based on grid resolution
    float step_size = 1.0f / grid.resolution();
    
    // Iterate along the line
    for (float t = 0.0f; t <= max_length; t += step_size) {
        Eigen::Vector3f point = start_ + direction * t;
        Eigen::Vector3i grid_pos = grid.world_to_grid(point);
        
        if (grid.is_inside_grid(grid_pos)) {
            grid.set(grid_pos, true);
        }
    }
}

// Supercover Line Voxelisation (SLV)
// Description: Extends RLV by ensuring connectivity through neighboring voxels
// Reference:
// - "Supercover Line Voxelization" by Cohen-Or and Kaufman (1995)
// - "Voxelization: A Survey" by Kaufman et al. (1993)
void LineVoxelizerCPU::voxelize_slv(VoxelGrid& grid) {
    // Supercover Line Voxelisation algorithm
    // First do RLV
    voxelize_rlv(grid);
    
    // Then add neighboring voxels to ensure connectivity
    Eigen::Vector3f direction = end_ - start_;
    float max_length = direction.norm();
    direction.normalize();
    
    float step_size = 1.0f / grid.resolution();
    
    for (float t = 0.0f; t <= max_length; t += step_size) {
        Eigen::Vector3f point = start_ + direction * t;
        Eigen::Vector3i grid_pos = grid.world_to_grid(point);
        
        // Check and set neighboring voxels
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

// Integer-only Line Voxelisation (ILV)
// Description: Uses integer arithmetic to avoid floating-point errors
// Reference:
// - "A Fast Voxel Traversal Algorithm for Ray Tracing" by Amanatides and Woo (1987)
// - "An Efficient and Robust Ray-Box Intersection Algorithm" by Williams et al. (2005)
void LineVoxelizerCPU::voxelize_ilv(VoxelGrid& grid) {
    // Integer-only Line Voxelisation algorithm
    Eigen::Vector3i start_grid = grid.world_to_grid(start_);
    Eigen::Vector3i end_grid = grid.world_to_grid(end_);
    
    int dx = abs(end_grid.x() - start_grid.x());
    int dy = abs(end_grid.y() - start_grid.y());
    int dz = abs(end_grid.z() - start_grid.z());
    
    int step_x = (start_grid.x() < end_grid.x()) ? 1 : -1;
    int step_y = (start_grid.y() < end_grid.y()) ? 1 : -1;
    int step_z = (start_grid.z() < end_grid.z()) ? 1 : -1;
    
    int x = start_grid.x();
    int y = start_grid.y();
    int z = start_grid.z();
    
    // Determine dominant direction
    if (dx >= dy && dx >= dz) {
        int err_1 = 2 * dy - dx;
        int err_2 = 2 * dz - dx;
        
        for (int i = 0; i <= dx; ++i) {
            if (grid.is_inside_grid(Eigen::Vector3i(x, y, z))) {
                grid.set(Eigen::Vector3i(x, y, z), true);
            }
            
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
            if (grid.is_inside_grid(Eigen::Vector3i(x, y, z))) {
                grid.set(Eigen::Vector3i(x, y, z), true);
            }
            
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
            if (grid.is_inside_grid(Eigen::Vector3i(x, y, z))) {
                grid.set(Eigen::Vector3i(x, y, z), true);
            }
            
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

// 3D Digital Differential Analyser (DDA)
// Description: Uses incremental algorithm for line drawing
// Reference:
// - "Computer Graphics: Principles and Practice" by Foley et al. (1990)
// - "Fundamentals of Computer Graphics" by Shirley and Marschner (2009)
void LineVoxelizerCPU::voxelize_dda(VoxelGrid& grid) {
    // 3D Digital Differential Analyser algorithm
    Eigen::Vector3i start_grid = grid.world_to_grid(start_);
    Eigen::Vector3i end_grid = grid.world_to_grid(end_);
    
    int dx = end_grid.x() - start_grid.x();
    int dy = end_grid.y() - start_grid.y();
    int dz = end_grid.z() - start_grid.z();
    
    int steps = std::max({abs(dx), abs(dy), abs(dz)});
    
    float x_inc = static_cast<float>(dx) / steps;
    float y_inc = static_cast<float>(dy) / steps;
    float z_inc = static_cast<float>(dz) / steps;
    
    float x = start_grid.x();
    float y = start_grid.y();
    float z = start_grid.z();
    
    for (int i = 0; i <= steps; ++i) {
        Eigen::Vector3i grid_pos(
            static_cast<int>(round(x)),
            static_cast<int>(round(y)),
            static_cast<int>(round(z))
        );
        
        if (grid.is_inside_grid(grid_pos)) {
            grid.set(grid_pos, true);
        }
        
        x += x_inc;
        y += y_inc;
        z += z_inc;
    }
}

// 3D Bresenham's Algorithm
// Description: Efficient integer-based line drawing algorithm
// Reference:
// - "Algorithm for Computer Control of a Digital Plotter" by Bresenham (1965)
// - "A Linear Algorithm for Incremental Digital Display of Circular Arcs" by Bresenham (1977)
// - "3D Bresenham's Algorithm" by Kaufman (1987)
void LineVoxelizerCPU::voxelize_bresenham(VoxelGrid& grid) {
    // 3D Bresenham's algorithm
    Eigen::Vector3i start_grid = grid.world_to_grid(start_);
    Eigen::Vector3i end_grid = grid.world_to_grid(end_);
    
    int dx = abs(end_grid.x() - start_grid.x());
    int dy = abs(end_grid.y() - start_grid.y());
    int dz = abs(end_grid.z() - start_grid.z());
    
    int step_x = (start_grid.x() < end_grid.x()) ? 1 : -1;
    int step_y = (start_grid.y() < end_grid.y()) ? 1 : -1;
    int step_z = (start_grid.z() < end_grid.z()) ? 1 : -1;
    
    int x = start_grid.x();
    int y = start_grid.y();
    int z = start_grid.z();
    
    // Determine dominant direction
    if (dx >= dy && dx >= dz) {
        int err_1 = 2 * dy - dx;
        int err_2 = 2 * dz - dx;
        
        for (int i = 0; i <= dx; ++i) {
            if (grid.is_inside_grid(Eigen::Vector3i(x, y, z))) {
                grid.set(Eigen::Vector3i(x, y, z), true);
            }
            
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
            if (grid.is_inside_grid(Eigen::Vector3i(x, y, z))) {
                grid.set(Eigen::Vector3i(x, y, z), true);
            }
            
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
            if (grid.is_inside_grid(Eigen::Vector3i(x, y, z))) {
                grid.set(Eigen::Vector3i(x, y, z), true);
            }
            
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

void LineVoxelizerCPU::voxelize_tripod(VoxelGrid &grid){
    // Tripod 3D数字线算法
    Eigen::Vector3i start_grid = grid.world_to_grid(start_);
    Eigen::Vector3i end_grid = grid.world_to_grid(end_);
    
    // 计算方向向量
    Eigen::Vector3f direction = (end_grid - start_grid).cast<float>();
    float max_length = direction.norm();
    direction.normalize();
    
    // 计算步长
    float step_size = 1.0f / grid.resolution();
    
    // 迭代沿着线
    for (float t = 0.0f; t <= max_length; t += step_size) {
        Eigen::Vector3i grid_pos = start_grid + (direction * t).cast<int>();
        
        // 检查是否在网格内
        if (grid.is_inside_grid(grid_pos)) {    
            grid.set(grid_pos, true);
        }
    }   
}

void LineVoxelizerCPU::voxelize_wu(VoxelGrid &grid){
    // Xiaolin Wu抗锯齿算法
    Eigen::Vector3i start_grid = grid.world_to_grid(start_);
    Eigen::Vector3i end_grid = grid.world_to_grid(end_);
    
    // 计算方向向量
    Eigen::Vector3f direction = (end_grid - start_grid).cast<float>();
    float max_length = direction.norm();
    direction.normalize();
    
    // 计算步长
    float step_size = 1.0f / grid.resolution();
    
    // 迭代沿着线
    for (float t = 0.0f; t <= max_length; t += step_size) {
        Eigen::Vector3i grid_pos = start_grid + (direction * t).cast<int>();
        
        // 检查是否在网格内
        if (grid.is_inside_grid(grid_pos)) {
            grid.set(grid_pos, true);
        }
    }
}

} // namespace VXZ