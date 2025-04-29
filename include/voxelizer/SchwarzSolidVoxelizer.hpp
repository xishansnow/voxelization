#pragma once

#include "voxelizer/voxelizer_base.hpp"
#include <vector>
#include <eigen3/Eigen/Core>

namespace VXZ {

// 基于 Schwarz 算法的实体体素化
class SchwarzSolidVoxelizer : public VoxelizerBase {
public:
    SchwarzSolidVoxelizer();

    // 设置输入多边形网格
    void set_mesh(const std::vector<Eigen::Vector3f>& vertices,
                  const std::vector<Eigen::Vector3i>& faces);
    // 体素化主接口
    bool voxelize(VoxelGrid& grid) const;

    bool ray_triangle_intersection(const Eigen::Vector3f &ray_origin, const Eigen::Vector3f &v0, const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, float &z) const;

    bool is_point_inside(const Eigen::Vector3f &point) const;

private:
    std::vector<Eigen::Vector3f> vertices_;
    std::vector<Eigen::Vector3i> faces_;
};

}
