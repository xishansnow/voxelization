#include "voxelizer/triangle_mesh_voxelizer.hpp"
#include <cmath>
#include <algorithm>


// 三角网格体素化算法
// 描述: 使用射线投射法判断体素是否在三角网格内部
// 参考文献:
// - "A Simple and Robust Method for Generating Solid Voxel Models of 3D Objects" by Kaufman et al. (1990)
// - "Fast 3D Triangle-Box Overlap Testing" by Akenine-Möller (2001)
// - "Real-Time Voxelization for Complex Polygonal Models" by Dong et al. (2004)
// - "Efficient GPU-Based Voxelization Using Hierarchical Occlusion Maps" by Schwarz and Seidel (2010)
// Dong的三角网格体素化算法
// 描述: 使用分离轴定理(SAT)和包围盒重叠测试进行快速体素化
// 参考文献:
// - "Real-Time Voxelization for Complex Polygonal Models" by Dong et al. (2004)



namespace VXZ{

// Kaufman的三角网格体素化算法
// 描述: 使用射线投射法判断体素是否在三角网格内部
// 参考文献:
// - "A Simple and Robust Method for Generating Solid Voxel Models of 3D Objects" by Kaufman et al. (1990)
void TriangleMeshVoxelizerCPU::voxelize_kaufman(VoxelGrid& grid) {
    // 遍历每个体素
    for (int x = 0; x < grid.dimensions().x(); ++x) {
        for (int y = 0; y < grid.dimensions().y(); ++y) {
            for (int z = 0; z < grid.dimensions().z(); ++z) {
                Eigen::Vector3i voxel_pos(x, y, z);
                Eigen::Vector3f world_pos = grid.grid_to_world(voxel_pos);
                
                // 射线投射方向(使用x轴方向)
                Eigen::Vector3f ray_dir(1.0f, 0.0f, 0.0f);
                
                // 统计射线与三角形的交点数
                int intersections = 0;
                float t;
                
                // 遍历所有三角形
                for (const auto& triangle : triangles_) {
                    if (ray_triangle_intersection(world_pos, ray_dir, triangle, t)) {
                        // 只统计射线正向的交点
                        if (t > 0) {
                            intersections++;
                        }
                    }
                }
                
                // 如果交点数为奇数,说明点在模型内部
                if (intersections % 2 == 1) {
                    grid.set(voxel_pos, true);
                }
            }
        }
    }
}

// 射线-三角形相交测试
// 使用Möller–Trumbore算法
// 参考文献:
// - "Fast, Minimum Storage Ray/Triangle Intersection" by Möller and Trumbore (1997)
bool TriangleMeshVoxelizerCPU::ray_triangle_intersection(
    const Eigen::Vector3f& origin,
    const Eigen::Vector3f& direction,
    const Triangle& triangle,
    float& t) const {
    
    // 计算边向量
    Eigen::Vector3f edge1 = triangle.v1 - triangle.v0;
    Eigen::Vector3f edge2 = triangle.v2 - triangle.v0;
    
    // 计算行列式的分量
    Eigen::Vector3f pvec = direction.cross(edge2);
    float det = edge1.dot(pvec);
    
    // 如果行列式接近0,射线与三角形平行
    if (std::abs(det) < 1e-8f) {
        return false;
    }
    
    float inv_det = 1.0f / det;
    
    // 计算从v0到射线起点的向量
    Eigen::Vector3f tvec = origin - triangle.v0;
    
    // 计算重心坐标u
    float u = tvec.dot(pvec) * inv_det;
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    
    // 计算重心坐标v
    Eigen::Vector3f qvec = tvec.cross(edge1);
    float v = direction.dot(qvec) * inv_det;
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    
    // 计算交点参数t
    t = edge2.dot(qvec) * inv_det;
    
    return true;
}



// bool TriangleMeshVoxelizerCPU::ray_triangle_intersection(
//     const Eigen::Vector3f& origin,
//     const Eigen::Vector3f& direction,
//     const Triangle& triangle,
//     float& t) const {
    
//     const float EPSILON = 1e-6f;
    
//     Eigen::Vector3f edge1 = triangle.v1 - triangle.v0;
//     Eigen::Vector3f edge2 = triangle.v2 - triangle.v0;
    
//     Eigen::Vector3f h = direction.cross(edge2);
//     float a = edge1.dot(h);
    
//     if (std::abs(a) < EPSILON) {
//         return false; // Ray is parallel to triangle
//     }
    
//     float f = 1.0f / a;
//     Eigen::Vector3f s = origin - triangle.v0;
//     float u = f * s.dot(h);
    
//     if (u < 0.0f || u > 1.0f) {
//         return false;
//     }
    
//     Eigen::Vector3f q = s.cross(edge1);
//     float v = f * direction.dot(q);
    
//     if (v < 0.0f || u + v > 1.0f) {
//         return false;
//     }
    
//     t = f * edge2.dot(q);
//     return t > EPSILON;
// }


void TriangleMeshVoxelizerCPU::voxelize(VoxelGrid& grid) {
    // 遍历每个三角形
    for (const auto& triangle : triangles_) {
        // 计算三角形的轴向包围盒(AABB)
        Eigen::Vector3f bbox_min = triangle.v0;
        Eigen::Vector3f bbox_max = triangle.v0;
        
        bbox_min = bbox_min.cwiseMin(triangle.v1);
        bbox_min = bbox_min.cwiseMin(triangle.v2);
        bbox_max = bbox_max.cwiseMax(triangle.v1); 
        bbox_max = bbox_max.cwiseMax(triangle.v2);
        
        // 将包围盒转换到网格坐标系
        Eigen::Vector3i grid_min = grid.world_to_grid(bbox_min);
        Eigen::Vector3i grid_max = grid.world_to_grid(bbox_max);
        
        // 确保在网格范围内
        grid_min = grid_min.cwiseMax(Eigen::Vector3i::Zero());
        grid_max = grid_max.cwiseMin(grid.dimensions() - Eigen::Vector3i::Ones());
        
        // 遍历包围盒内的所有体素
        for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
            for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
                for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                    Eigen::Vector3i voxel_pos(x, y, z);
                    
                    // 获取体素的世界坐标边界
                    Eigen::Vector3f voxel_min = grid.grid_to_world(voxel_pos);
                    Eigen::Vector3f voxel_max = grid.grid_to_world(voxel_pos + Eigen::Vector3i::Ones());
                    
                    // 检查三角形和体素是否重叠
                    if (triangle_voxel_overlap(triangle, voxel_min, voxel_max)) {
                        grid.set(voxel_pos, true);
                    }
                }
            }
        }
    }
}

// 判断三角形和体素是否重叠的辅助函数
bool TriangleMeshVoxelizerCPU::triangle_voxel_overlap(const Triangle & triangle, const Eigen::Vector3f& voxel_min, const Eigen::Vector3f& voxel_max) {
    // 1. 分离轴测试 - 体素的三个面法线方向
    Eigen::Vector3f voxel_center = (voxel_min + voxel_max) * 0.5f;
    Eigen::Vector3f voxel_half_size = (voxel_max - voxel_min) * 0.5f;
    // 三角形顶点相对于体素中心的位置
    Eigen::Vector3f v0 = triangle.v0 - voxel_center;
    Eigen::Vector3f v1 = triangle.v1 - voxel_center;
    Eigen::Vector3f v2 = triangle.v2 - voxel_center;
    
    // 检查三角形投影是否与体素重叠
    for(int i = 0; i < 3; i++) {
        float p0 = v0[i];
        float p1 = v1[i];
        float p2 = v2[i];
        float r = voxel_half_size[i];
        
        if(std::min({p0, p1, p2}) > r || std::max({p0, p1, p2}) < -r) {
            return false;
        }
    }
    
    // 2. 分离轴测试 - 三角形法线方向
    float p0 = triangle.normal.dot(v0);
    float p1 = triangle.normal.dot(v1);
    float p2 = triangle.normal.dot(v2);
    float r = voxel_half_size.cwiseAbs().dot(triangle.normal.cwiseAbs());
    
    if(std::min({p0, p1, p2}) > r || std::max({p0, p1, p2}) < -r) {
        return false;
    }
    
    // 3. 分离轴测试 - 三角形边与体素面法线的叉积
    Eigen::Vector3f edge0 = v1 - v0;
    Eigen::Vector3f edge1 = v2 - v1;
    Eigen::Vector3f edge2 = v0 - v2;
    
    for(int i = 0; i < 3; i++) {
        Eigen::Vector3f axis = Eigen::Vector3f::Unit(i).cross(edge0);
        float r = voxel_half_size.cwiseAbs().dot(axis.cwiseAbs());
        float p0 = axis.dot(v0);
        float p1 = axis.dot(v1);
        float p2 = axis.dot(v2);
        
        if(std::min({p0, p1, p2}) > r || std::max({p0, p1, p2}) < -r) {
            return false;
        }
    }
    
    return true;
}



void TriangleMeshVoxelizerGPU::voxelize(VoxelGrid& grid) {

} 
}// namespace VXZ 