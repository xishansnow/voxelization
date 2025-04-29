import os

# 1. 目录准备
include_dir = "include/voxelizer"
src_dir = "src/voxelizer"
test_dir = "tests"
os.makedirs(include_dir, exist_ok=True)
os.makedirs(src_dir, exist_ok=True)
os.makedirs(test_dir, exist_ok=True)

# 2. 类定义
solid_voxelizers = [
    {
        "name": "SchwarzSolidVoxelizer",
        "desc": "基于 Schwarz 算法的实体体素化",
        "comment": "Schwarz 算法体素化实现（占位）\n    // 典型做法：对每个体素中心点，沿主轴方向投影，统计交点数，奇偶性判定内外",
        "todo": "// TODO: 实现 Schwarz 算法的射线投影与奇偶性判定"
    },
    {
        "name": "EisemannSolidVoxelizer",
        "desc": "基于 Eisemann 算法的实体体素化",
        "comment": "Eisemann 算法体素化实现（占位）\n    // 典型做法：基于多方向投影和修正，处理薄壳和复杂拓扑",
        "todo": "// TODO: 实现 Eisemann 算法的多方向投影与修正"
    }
]

# 3. 生成头文件和实现文件
for cls in solid_voxelizers:
    hpp_path = os.path.join(include_dir, f"{cls['name']}.hpp")
    cpp_path = os.path.join(src_dir, f"{cls['name']}.cpp")
    # 头文件
    with open(hpp_path, "w") as f:
        f.write(f"""#pragma once
#include "voxelizer/voxelizer_base.hpp"
#include <vector>
#include <Eigen/Dense>

namespace VXZ {{

// {cls['desc']}
class {cls['name']} : public VoxelizerBase {{
public:
    {cls['name']}();

    // 设置输入多边形网格
    void set_mesh(const std::vector<Eigen::Vector3f>& vertices,
                  const std::vector<Eigen::Vector3i>& faces);

    // 体素化主接口
    bool voxelize(VoxelGrid& grid) const override;

private:
    std::vector<Eigen::Vector3f> vertices_;
    std::vector<Eigen::Vector3i> faces_;
}};

}}
""")
    # 实现文件
    with open(cpp_path, "w") as f:
        f.write(f"""#include "voxelizer/{cls['name']}.hpp"

namespace VXZ {{

{cls['name']}::{cls['name']}() {{}}

void {cls['name']}::set_mesh(const std::vector<Eigen::Vector3f>& vertices,
                             const std::vector<Eigen::Vector3i>& faces) {{
    vertices_ = vertices;
    faces_ = faces;
}}

bool {cls['name']}::voxelize(VoxelGrid& grid) const {{
    // {cls['comment']}
    for (int z = 0; z < grid.get_size_z(); ++z)
        for (int y = 0; y < grid.get_size_y(); ++y)
            for (int x = 0; x < grid.get_size_x(); ++x) {{
                {cls['todo']}
                grid.set(x, y, z, false);
            }}
    return true;
}}

}}
""")

# 4. 生成测试用例
test_path = os.path.join(test_dir, "solid_voxelizer_test.cpp")
with open(test_path, "w") as f:
    f.write("""#include <gtest/gtest.h>
#include <voxelizer/SchwarzSolidVoxelizer.hpp>
#include <voxelizer/EisemannSolidVoxelizer.hpp>
#include <core/voxel_grid.hpp>
#include <vector>

using namespace VXZ;

static std::vector<Eigen::Vector3f> cube_vertices = {
    {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
    {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}
};
static std::vector<Eigen::Vector3i> cube_faces = {
    {{0,1,2}}, {{0,2,3}}, // bottom
    {{4,5,6}}, {{4,6,7}}, // top
    {{0,1,5}}, {{0,5,4}}, // front
    {{2,3,7}}, {{2,7,6}}, // back
    {{1,2,6}}, {{1,6,5}}, // right
    {{0,3,7}}, {{0,7,4}}  // left
};

TEST(SchwarzSolidVoxelizerTest, Cube) {
    VoxelGrid grid(0.5f, Eigen::Vector3f(0,0,0), Eigen::Vector3f(3,3,3));
    SchwarzSolidVoxelizer voxelizer;
    voxelizer.set_mesh(cube_vertices, cube_faces);
    ASSERT_TRUE(voxelizer.voxelize(grid));
    // 这里只检查接口可用
    EXPECT_EQ(grid.get_size_x(), 3);
}

TEST(EisemannSolidVoxelizerTest, Cube) {
    VoxelGrid grid(0.5f, Eigen::Vector3f(0,0,0), Eigen::Vector3f(3,3,3));
    EisemannSolidVoxelizer voxelizer;
    voxelizer.set_mesh(cube_vertices, cube_faces);
    ASSERT_TRUE(voxelizer.voxelize(grid));
    // 这里只检查接口可用
    EXPECT_EQ(grid.get_size_x(), 3);
}
""")

print("已批量生成 SchwarzSolidVoxelizer、EisemannSolidVoxelizer 及其测试用例。")