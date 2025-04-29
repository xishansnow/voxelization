#include "voxelizer/EisemannSolidVoxelizer.hpp"

namespace VXZ {

EisemannSolidVoxelizer::EisemannSolidVoxelizer() {}

void EisemannSolidVoxelizer::set_mesh(const std::vector<Eigen::Vector3f>& vertices,
                             const std::vector<Eigen::Vector3i>& faces) {
    vertices_ = vertices;
    faces_ = faces;
}

bool EisemannSolidVoxelizer::voxelize(VoxelGrid& grid) const {
    // Eisemann 算法体素化实现（占位）
    // 典型做法：基于多方向投影和修正，处理薄壳和复杂拓扑
    for (int z = 0; z < grid.get_size_z(); ++z)
        for (int y = 0; y < grid.get_size_y(); ++y)
            for (int x = 0; x < grid.get_size_x(); ++x) {
                // 获取体素中心点的世界坐标
                Eigen::Vector3f pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                // 调用 is_point_inside 判断点是否在内部
                bool is_inside = is_point_inside(pos);
                // TODO: 实现 Eisemann 算法的多方向投影与修正
                grid.set(x, y, z, is_inside);
            }
    return true;
}

// 判断射线与三角形是否相交,返回交点的参数 t
bool EisemannSolidVoxelizer::ray_triangle_intersection(const Eigen::Vector3f& ray_origin,
                             const Eigen::Vector3f& ray_dir,
                             const Eigen::Vector3f& v0,
                             const Eigen::Vector3f& v1,
                             const Eigen::Vector3f& v2,
                             float& t) const {
    // 计算三角形法向量
    Eigen::Vector3f edge1 = v1 - v0;
    Eigen::Vector3f edge2 = v2 - v0;
    Eigen::Vector3f normal = edge1.cross(edge2);

    // 如果三角形近乎平行于射线,则忽略
    float dot = normal.dot(ray_dir);
    if(std::abs(dot) < 1e-6f) {
        return false;
    }

    // 计算交点参数
    t = -(normal.dot(ray_origin - v0)) / dot;
    
    // 交点在射线反方向,则忽略
    if(t < 0) {
        return false;
    }

    // 计算交点
    Eigen::Vector3f p = ray_origin + t * ray_dir;

    // 判断交点是否在三角形内
    Eigen::Vector3f c;

    // 边界测试
    c = (v1 - v0).cross(p - v0);
    if(normal.dot(c) < 0) return false;

    c = (v2 - v1).cross(p - v1);
    if(normal.dot(c) < 0) return false;

    c = (v0 - v2).cross(p - v2);
    if(normal.dot(c) < 0) return false;

    return true;
}

// 判断点是否在实体内部(多方向投影)
bool EisemannSolidVoxelizer::is_point_inside(const Eigen::Vector3f& point) const {
    // 定义6个主轴方向
    const Eigen::Vector3f dirs[6] = {
        Eigen::Vector3f(1,0,0), Eigen::Vector3f(-1,0,0),
        Eigen::Vector3f(0,1,0), Eigen::Vector3f(0,-1,0),
        Eigen::Vector3f(0,0,1), Eigen::Vector3f(0,0,-1)
    };
    
    int inside_count = 0;

    // 对每个方向进行射线测试
    for(int d = 0; d < 6; d++) {
        int intersect_count = 0;
        float t;

        // 对每个三角形面片进行射线测试
        for(const auto& face : faces_) {
            const auto& v0 = vertices_[face[0]];
            const auto& v1 = vertices_[face[1]];
            const auto& v2 = vertices_[face[2]];

            if(ray_triangle_intersection(point, dirs[d], v0, v1, v2, t)) {
                // 只统计射线正方向的交点
                if(t > 0) {
                    intersect_count++;
                }
            }
        }

        // 根据交点数的奇偶性判断该方向的内外性
        if((intersect_count % 2) == 1) {
            inside_count++;
        }
    }

    // 如果大多数方向判定为内部,则认为点在内部
    return inside_count > 3;
}


}
