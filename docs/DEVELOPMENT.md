# Voxelization 开发文档

## 项目概述

Voxelization 是一个用于体素化（Voxelization）的 C++ 库，支持多种体素化方法和存储格式。该项目提供了灵活的体素化工具，可以处理各种几何形状，并支持多种存储格式。

## 目录结构

```
Voxelization/
├── include/                 # 头文件
│   ├── core/               # 核心数据结构
│   ├── voxelizer/          # 体素化器
│   ├── storage/            # 存储格式
│   ├── operator/           # 网格操作
│   └── renderer/           # 渲染器
├── src/                    # 源文件
│   ├── core/               # 核心实现
│   ├── voxelizer/          # 体素化器实现
│   ├── storage/            # 存储格式实现
│   ├── operator/           # 网格操作实现
│   └── renderer/           # 渲染器实现
├── tests/                  # 测试文件
├── examples/               # 示例代码
└── docs/                   # 文档
```

## 主要组件

### 1. 核心组件 (Core)

- `VoxelGrid`: 基础体素网格数据结构
  - 支持创建、访问和修改体素网格
  - 提供基本的网格操作功能

### 2. 体素化器 (Voxelizer)

支持多种几何形状的体素化：

- `BoxVoxelizer`: 立方体体素化
- `SphereVoxelizer`: 球体体素化
- `CylinderVoxelizer`: 圆柱体素化
- `PointCloudVoxelizer`: 点云体素化
- `PolyhedronVoxelizer`: 多面体体素化
- `TriangleMeshVoxelizer`: 三角网格体素化
- `CorridorVoxelizer`: 走廊体素化
- `LineVoxelizer`: 线段体素化
- `PolylineVoxelizer`: 折线体素化
- `SplineVoxelizer`: 样条曲线体素化
- `MultiLevelVoxelGrid`: 多级体素网格

### 3. 存储格式 (Storage)

支持多种存储格式：

- `SVOStorage`: 稀疏体素八叉树存储
- `SVDAGStorage`: 稀疏体素有向无环图存储
- `SSVDAGStorage`: 共享稀疏体素有向无环图存储
- `OpenVDBStorage`: OpenVDB 格式存储

### 4. 网格操作 (Operator)

提供多种网格操作：

- `SmoothOperator`: 平滑操作
- `DilateOperator`: 膨胀操作
- `ErodeOperator`: 腐蚀操作
- `OffsetOperator`: 偏移操作
- `UnionOperator`: 并集操作
- `IntersectionOperator`: 交集操作
- `DifferenceOperator`: 差集操作
- `OpeningOperator`: 开运算
- `ClosingOperator`: 闭运算
- `DistanceTransformOperator`: 距离变换
- `ConnectedComponentsOperator`: 连通区域标记
- `FillOperator`: 填充操作

### 5. 渲染器 (Renderer)

- `VoxelRenderer`: 体素渲染器
  - 支持 OpenGL 渲染
  - 提供基本的交互功能（旋转、平移、缩放）

## 构建系统

项目使用 CMake 作为构建系统。主要配置选项：

```cmake
option(BUILD_TESTS "Build tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)
```

### 依赖项

项目依赖以下库：
- Eigen3
- OpenMP
- OpenVDB
- GTest
- OpenGL
- GLEW
- GLFW3

详细的依赖项安装说明请参考 [DEPENDENCIES.md](DEPENDENCIES.md)。

## 使用示例

### 基本体素化

```cpp
#include "core/voxel_grid.hpp"
#include "voxelizer/box_voxelizer.hpp"

int main() {
    // 创建体素网格
    VoxelGrid grid(1000);
    
    // 创建体素化器
    BoxVoxelizer voxelizer;
    
    // 设置体素化参数
    Box box;
    box.min = Eigen::Vector3f(0, 0, 0);
    box.max = Eigen::Vector3f(100, 100, 100);
    
    // 执行体素化
    voxelizer.voxelize(grid, box);
    
    return 0;
}
```

### 存储和加载

```cpp
#include "storage/svo.hpp"

int main() {
    // 创建存储对象
    SVOStorage storage;
    
    // 从体素网格创建存储
    storage.fromVoxelGrid(grid);
    
    // 保存到文件
    storage.save("output.svo");
    
    // 从文件加载
    storage.load("input.svo");
    
    // 转换回体素网格
    VoxelGrid newGrid;
    storage.toVoxelGrid(newGrid);
    
    return 0;
}
```

### 网格操作

```cpp
#include "operator/grid_operator.hpp"

int main() {
    // 创建操作器
    SmoothOperator smoothOp(3, 0.5f);
    DilateOperator dilateOp(2);
    
    // 应用操作
    smoothOp.apply(grid);
    dilateOp.apply(grid);
    
    return 0;
}
```

## 测试

项目包含完整的测试套件，使用 Google Test 框架。测试文件位于 `tests/` 目录下。

运行测试：
```bash
cd build
cmake ..
make
ctest
```

## 贡献指南

1. 遵循项目的代码风格
2. 添加适当的测试用例
3. 更新文档
4. 提交 Pull Request

## 许可证

本项目采用 MIT 许可证。详见 LICENSE 文件。 