#include "voxelizer/SchwarzSolidVoxelizer.hpp"




namespace VXZ {

SchwarzSolidVoxelizer::SchwarzSolidVoxelizer() {}

void SchwarzSolidVoxelizer::set_mesh(const std::vector<Eigen::Vector3f>& vertices,
                             const std::vector<Eigen::Vector3i>& faces) {
    vertices_ = vertices;
    faces_ = faces;
}

bool SchwarzSolidVoxelizer::voxelize(VoxelGrid& grid) const {
    // Schwarz 算法体素化实现（占位）
    // 典型做法：对每个体素中心点，沿主轴方向投影，统计交点数，奇偶性判定内外
    for (int z = 0; z < grid.get_size_z(); ++z)
        for (int y = 0; y < grid.get_size_y(); ++y)
            for (int x = 0; x < grid.get_size_x(); ++x) {
                // TODO: 实现 Schwarz 算法的射线投影与奇偶性判定
                // 获取体素中心点的世界坐标
                Eigen::Vector3f pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                // 调用 is_point_inside 判断点是否在内部
                bool is_inside = is_point_inside(pos);
                // 更新体素值
                grid.set(x, y, z, is_inside);                
            }
    return true;
}

// 判断射线与三角形是否相交,返回交点的 z 坐标
bool SchwarzSolidVoxelizer::ray_triangle_intersection(const Eigen::Vector3f& ray_origin,
                             const Eigen::Vector3f& v0,
                             const Eigen::Vector3f& v1, 
                             const Eigen::Vector3f& v2,
                             float& z) const {
    // 计算三角形法向量
    Eigen::Vector3f edge1 = v1 - v0;
    Eigen::Vector3f edge2 = v2 - v0;
    Eigen::Vector3f normal = edge1.cross(edge2);

    // 如果三角形近乎垂直于射线,则忽略
    if(std::abs(normal.z()) < 1e-6f) {
        return false;
    }

    // 射线方向固定为 (0,0,1)
    const Eigen::Vector3f ray_dir(0, 0, 1);
    
    // 计算交点参数
    float t = -(normal.dot(ray_origin - v0)) / normal.dot(ray_dir);
    
    // 交点在射线反方向,则忽略
    if(t < 0) {
        return false;
    }

    // 计算交点
    Eigen::Vector3f p = ray_origin + t * ray_dir;

    // 判断交点是否在三角形内
    Eigen::Vector3f c;

    // 边1测试
    c = (v1 - v0).cross(p - v0);
    if(normal.dot(c) < 0) return false;

    // 边2测试
    c = (v2 - v1).cross(p - v1);
    if(normal.dot(c) < 0) return false;

    // 边3测试
    c = (v0 - v2).cross(p - v2);
    if(normal.dot(c) < 0) return false;

    z = p.z();
    return true;
}

// 判断点是否在实体内部
bool SchwarzSolidVoxelizer::is_point_inside(const Eigen::Vector3f& point) const {
    int intersect_count = 0;
    float z;

    // 对每个三角形面片进行射线测试
    for(const auto& face : faces_) {
        const auto& v0 = vertices_[face[0]];
        const auto& v1 = vertices_[face[1]]; 
        const auto& v2 = vertices_[face[2]];

        if(ray_triangle_intersection(point, v0, v1, v2, z)) {
            // 只统计射线正方向的交点
            if(z > point.z()) {
                intersect_count++;
            }
        }
    }

    // 根据交点数的奇偶性判断内外
    return (intersect_count % 2) == 1;
}


}
