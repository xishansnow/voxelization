# 使用示例

本文档提供体素化库的实际使用示例。

## 基本用法

### 创建体素网格

```cpp
#include "core/voxel_grid.hpp"

// 创建分辨率为 0.1 米的体素网格
VoxelGrid grid(0.1f,                    // 分辨率
              Eigen::Vector3f(-1, -1, -1),  // 最小边界
              Eigen::Vector3f(1, 1, 1),     // 最大边界
              false);                    // 使用 CPU
```

### 体素化盒子

```cpp
#include "voxelizer/box_voxelizer.hpp"

// 创建盒子体素化器
auto box_voxelizer = BoxVoxelizer::create(
    Eigen::Vector3f(0, 0, 0),  // 中心
    Eigen::Vector3f(1, 1, 1),  // 尺寸
    false                      // 使用 CPU
);

// 体素化盒子
box_voxelizer->voxelize(grid);
```

### 体素化球体

```cpp
#include "voxelizer/sphere_voxelizer.hpp"

// 创建球体体素化器
auto sphere_voxelizer = SphereVoxelizer::create(
    Eigen::Vector3f(0, 0, 0),  // 中心
    1.0f,                      // 半径
    false                      // 使用 CPU
);

// 体素化球体
sphere_voxelizer->voxelize(grid);
```

## 高级用法

### 体素化直线

```cpp
#include "voxelizer/line_voxelizer.hpp"

// 创建使用 Bresenham 算法的直线体素化器
auto line_voxelizer = LineVoxelizer::create(
    Eigen::Vector3f(0, 0, 0),  // 起点
    Eigen::Vector3f(1, 1, 1),  // 终点
    false,                     // 使用 CPU
    LineVoxelizerCPU::Algorithm::BRESENHAM
);

// 体素化直线
line_voxelizer->voxelize(grid);
```

### 体素化折线

```cpp
#include "voxelizer/polyline_voxelizer.hpp"

// 创建折线
std::vector<Eigen::Vector3f> points = {
    Eigen::Vector3f(0, 0, 0),
    Eigen::Vector3f(1, 0, 0),
    Eigen::Vector3f(1, 1, 0),
    Eigen::Vector3f(0, 1, 0)
};

// 创建折线体素化器
auto polyline_voxelizer = PolylineVoxelizer::create(
    points,                    // 点集
    false,                     // 使用 CPU
    LineVoxelizerCPU::Algorithm::BRESENHAM
);

// 体素化折线
polyline_voxelizer->voxelize(grid);
```

### 样条曲线体素化

#### 创建样条曲线体素化器

```cpp
// 创建样条曲线的控制点
std::vector<Eigen::Vector3f> control_points = {
    {0, 0, 0},
    {1, 1, 0},
    {2, 0, 1},
    {3, 1, 1}
};

// 创建 Catmull-Rom 样条体素化器
auto voxelizer = SplineVoxelizer::create(control_points, 0.1f, 0, false);

// 创建体素网格
VoxelGrid grid({100, 100, 100}, {0, 0, 0}, {10, 10, 10});

// 体素化样条曲线
voxelizer->voxelize(grid);
```

#### 使用不同的样条类型

```cpp
// 创建 B 样条体素化器
auto bspline_voxelizer = SplineVoxelizer::create(control_points, 0.1f, 1, false);

// 创建贝塞尔曲线体素化器
auto bezier_voxelizer = SplineVoxelizer::create(control_points, 0.1f, 2, false);
```

#### GPU 加速

```cpp
// 创建 GPU 加速的样条曲线体素化器
auto gpu_voxelizer = SplineVoxelizer::create(control_points, 0.1f, 0, true);

// 创建大型体素网格
VoxelGrid large_grid({500, 500, 500}, {0, 0, 0}, {10, 10, 10});

// 使用 GPU 进行体素化
gpu_voxelizer->voxelize(large_grid);
```

#### 错误处理

```cpp
try {
    // 尝试使用无效参数创建样条曲线体素化器
    std::vector<Eigen::Vector3f> invalid_points = {{0, 0, 0}};  // 控制点太少
    auto voxelizer = SplineVoxelizer::create(invalid_points, 0.1f, 0, false);
} catch (const std::invalid_argument& e) {
    std::cerr <<"错误:" << e.what() << std::endl;
}
```

## GPU 用法

### 创建 GPU 体素网格

```cpp
// 创建支持 GPU 的体素网格
VoxelGrid grid(0.1f,                    // 分辨率
              Eigen::Vector3f(-1, -1, -1),  // 最小边界
              Eigen::Vector3f(1, 1, 1),     // 最大边界
              true);                     // 使用 GPU
```

### 使用 GPU 体素化

```cpp
// 创建支持 GPU 的盒子体素化器
auto box_voxelizer = BoxVoxelizer::create(
    Eigen::Vector3f(0, 0, 0),  // 中心
    Eigen::Vector3f(1, 1, 1),  // 尺寸
    true                       // 使用 GPU
);

// 使用 GPU 体素化盒子
box_voxelizer->voxelize(grid);
```

## 网格操作

### 访问体素

```cpp
// 获取体素值
Eigen::Vector3i position(10, 10, 10);
bool value = grid.get(position);

// 设置体素值
grid.set(position, true);
```

### 批量操作

```cpp
// 填充整个网格
grid.fill(true);

// 清空整个网格
grid.clear();

// 设置网格区域
Eigen::Vector3i min(0, 0, 0);
Eigen::Vector3i max(10, 10, 10);
grid.set_region(min, max, true);
```

### 坐标转换

```cpp
// 将世界坐标转换为网格坐标
Eigen::Vector3f world_pos(0.5f, 0.5f, 0.5f);
Eigen::Vector3i grid_pos = grid.world_to_grid(world_pos);

// 将网格坐标转换为世界坐标
Eigen::Vector3f world_pos2 = grid.grid_to_world(grid_pos);
```

### 统计信息

```cpp
// 计算被占用的体素数量
size_t occupied = grid.count_occupied();

// 计算占用率
float rate = grid.occupancy_rate();
```

## 错误处理

```cpp
try {
    // 创建体素网格
    VoxelGrid grid(0.1f,
                  Eigen::Vector3f(-1, -1, -1),
                  Eigen::Vector3f(1, 1, 1),
                  true);

    // 体素化盒子
    auto box_voxelizer = BoxVoxelizer::create(
        Eigen::Vector3f(0, 0, 0),
        Eigen::Vector3f(1, 1, 1),
        true
    );
    box_voxelizer->voxelize(grid);

} catch (const std::invalid_argument& e) {
    // 处理无效参数
    std::cerr <<"无效参数:" << e.what() << std::endl;
} catch (const std::out_of_range& e) {
    // 处理越界错误
    std::cerr <<"越界:" << e.what() << std::endl;
} catch (const std::runtime_error& e) {
    // 处理运行时错误（包括 CUDA 错误）
    std::cerr <<"运行时错误:" << e.what() << std::endl;
}
```

## 性能提示

1. 对大型网格使用 GPU
2. 最小化 CPU-GPU 数据传输
3. 尽可能使用批量操作
4. 选择合适的分辨率
5. 尽可能重用体素网格