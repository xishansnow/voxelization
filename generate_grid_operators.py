import os

# 子类信息（类名, 构造参数, 额外方法声明, 额外方法实现, 头文件依赖）
subclasses = [
    ("SmoothOperator", "int iterations = 1, float threshold = 0.5f", "", "", ""),
    ("DilateOperator", "int iterations = 1", "", "", ""),
    ("ErodeOperator", "int iterations = 1", "", "", ""),
    ("OffsetOperator", "float distance = 1.0f", "", "", ""),
    ("UnionOperator", "const VXZ::VoxelGrid& other", "", "", ""),
    ("IntersectionOperator", "const VXZ::VoxelGrid& other", "", "", ""),
    ("DifferenceOperator", "const VXZ::VoxelGrid& other", "", "", ""),
    ("OpeningOperator", "int iterations = 1", "", "", ""),
    ("ClosingOperator", "int iterations = 1", "", "", ""),
    ("DistanceTransformOperator", "float max_distance = std::numeric_limits<float>::max()", "", "", "#include <limits>"),
    ("ConnectedComponentsOperator", "int connectivity = 6", "int get_component_count() const;", "int ConnectedComponentsOperator::get_component_count() const { return component_count_; }", ""),
    ("FillOperator", "const Eigen::Vector3i& seed_point, int connectivity = 6", "", "", "#include <eigen3/Eigen/Dense>"),
    ("InterpolationOperator", "const Eigen::Vector3f& position", "float get_value() const;", "float InterpolationOperator::get_value() const { return interpolated_value_; }", "#include <eigen3/Eigen/Dense>"),
]

hpp_dir = "include/operator"
cpp_dir = "src/operator"
os.makedirs(hpp_dir, exist_ok=True)
os.makedirs(cpp_dir, exist_ok=True)

# 生成每个子类的 .hpp 和 .cpp 文件
for name, ctor, extra_decl, extra_impl, extra_include in subclasses:
    hpp_path = os.path.join(hpp_dir, f"{name}.hpp")
    cpp_path = os.path.join(cpp_dir, f"{name}.cpp")
    with open(hpp_path, "w") as f:
        f.write(f"""#pragma once
#include "operator/grid_operator.hpp"
{extra_include}
namespace VXZ {{

class {name} : public GridOperator {{
public:
    explicit {name}({ctor});
    bool apply(VXZ::VoxelGrid& grid) const override;
    {extra_decl}
private:
    // 成员变量根据原始定义补充
}};

}}
""")
    with open(cpp_path, "w") as f:
        f.write(f"""#include "operator/{name}.hpp"
{extra_include}
namespace VXZ {{

{name}::{name}({ctor}) {{}}

bool {name}::apply(VXZ::VoxelGrid& grid) const {{
    // TODO: 实现 {name} 算法
    return true;
}}
{extra_impl if extra_impl and "get_value" in extra_impl else ""}
}}
""")

# 生成 grid_operator.hpp
grid_operator_hpp = os.path.join(hpp_dir, "grid_operator.hpp")
with open(grid_operator_hpp, "w") as f:
    f.write("""#pragma once

#include <memory>
#include <eigen3/Eigen/Dense>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range3d.h>
#include "../core/voxel_grid.hpp"
""")
    for name, *_ in subclasses:
        f.write(f'#include "operator/{name}.hpp"\n')
    f.write("""
namespace VXZ {

class GridOperator {
public:
    virtual ~GridOperator() = default;
    virtual bool apply(VXZ::VoxelGrid& grid) const = 0;

protected:
    template<typename Func>
    void parallel_apply(VXZ::VoxelGrid& grid, Func func) const {
        tbb::parallel_for(
            tbb::blocked_range3d<int>(0, grid.get_size_x(), 0, grid.get_size_y(), 0, grid.get_size_z()),
            [&](const tbb::blocked_range3d<int>& r) {
                for (int z = r.pages().begin(); z < r.pages().end(); ++z) {
                    for (int y = r.rows().begin(); y < r.rows().end(); ++y) {
                        for (int x = r.cols().begin(); x < r.cols().end(); ++x) {
                            func(x, y, z);
                        }
                    }
                }
            }
        );
    }
};

}
""")

print("所有子类 .hpp/.cpp 文件和 grid_operator.hpp 已批量生成。")
