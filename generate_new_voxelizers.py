import os

# 1. 目录准备
include_dir = "include/voxelizer"
src_dir = "src/voxelizer"
test_dir = "tests"
os.makedirs(include_dir, exist_ok=True)
os.makedirs(src_dir, exist_ok=True)
os.makedirs(test_dir, exist_ok=True)

# 2. 类定义
voxelizer_classes = [
    {
        "name": "LevelSetVoxelizer",
        "desc": "Level Set 体素化",
        "func": "level_set_function",
        "func_desc": "可扩展的 level set 函数接口",
        "default_impl": "return pos.norm() - 1.0f;",
        "header": "virtual float level_set_function(const Eigen::Vector3f& pos) const;",
        "call": "level_set_function(pos) <= 0.0f"
    },
    {
        "name": "SDFVoxelizer",
        "desc": "Signed Distance Field 体素化",
        "func": "sdf",
        "func_desc": "SDF函数接口",
        "default_impl": "return pos.norm() - 1.0f;",
        "header": "virtual float sdf(const Eigen::Vector3f& pos) const;",
        "call": "sdf(pos) <= 0.0f"
    },
    {
        "name": "ImplicitSurfaceVoxelizer",
        "desc": "Implicit Surface 体素化",
        "func": "implicit_function",
        "func_desc": "隐式面函数接口",
        "default_impl": "return pos.norm() - 1.0f;",
        "header": "virtual float implicit_function(const Eigen::Vector3f& pos) const;",
        "call": "implicit_function(pos) <= 0.0f"
    }
]

# 3. 生成头文件和实现文件
for cls in voxelizer_classes:
    hpp_path = os.path.join(include_dir, f"{cls['name']}.hpp")
    cpp_path = os.path.join(src_dir, f"{cls['name']}.cpp")
    # 头文件
    with open(hpp_path, "w") as f:
        f.write(f"""#pragma once
#include "voxelizer/voxelizer_base.hpp"

namespace VXZ {{

class {cls['name']} : public VoxelizerBase {{
public:
    {cls['name']}();

    // 体素化主接口
    bool voxelize(VoxelGrid& grid) const override;

    // {cls['func_desc']}
    {cls['header']}
}};

}}
""")
    # 实现文件
    with open(cpp_path, "w") as f:
        f.write(f"""#include "voxelizer/{cls['name']}.hpp"

namespace VXZ {{

{cls['name']}::{cls['name']}() {{}}

bool {cls['name']}::voxelize(VoxelGrid& grid) const {{
    for (int z = 0; z < grid.get_size_z(); ++z)
        for (int y = 0; y < grid.get_size_y(); ++y)
            for (int x = 0; x < grid.get_size_x(); ++x) {{
                Eigen::Vector3f pos = grid.index_to_world(x, y, z);
                grid.set(x, y, z, {cls['call']});
            }}
    return true;
}}

float {cls['name']}::{cls['func']}(const Eigen::Vector3f& pos) const {{
    // 默认实现：球体
    {cls['default_impl']}
}}

}}
""")

# 4. 生成测试用例
test_path = os.path.join(test_dir, "voxelizer_new_test.cpp")
with open(test_path, "w") as f:
    f.write("""#include <gtest/gtest.h>
#include <voxelizer/LevelSetVoxelizer.hpp>
#include <voxelizer/SDFVoxelizer.hpp>
#include <voxelizer/ImplicitSurfaceVoxelizer.hpp>
#include <core/voxel_grid.hpp>

using namespace VXZ;

""")
    for cls in voxelizer_classes:
        f.write(f"""TEST({cls['name']}Test, Sphere) {{
    VoxelGrid grid(1.0f, Eigen::Vector3f(0,0,0), Eigen::Vector3f(8,8,8));
    {cls['name']} voxelizer;
    ASSERT_TRUE(voxelizer.voxelize(grid));
    // 检查中心点是否为true
    EXPECT_TRUE(grid.get(4,4,4));
}}

""")

print("已批量生成 LevelSetVoxelizer、SDFVoxelizer、ImplicitSurfaceVoxelizer 及其测试用例。")