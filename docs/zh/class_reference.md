# 类参考

本文档提供体素化系统中所有类的详细 API 文档。

## 核心类

### VoxelizerBase

所有体素化器的基类。

```cpp
class VoxelizerBase {
public:
    virtual ~VoxelizerBase() = default;
    static std::unique_ptr<VoxelizerBase> create(bool use_gpu);
    virtual VoxelGrid voxelize(float resolution,
                             const Eigen::Vector3f& min_bounds,
                             const Eigen::Vector3f& max_bounds) = 0;
    bool is_gpu() const { return use_gpu_;}
protected:
    VoxelizerBase(bool use_gpu) : use_gpu_(use_gpu) {}
    bool use_gpu_;
};
```

### VoxelizerCPU

CPU 实现的基类。

```cpp
class VoxelizerCPU : public VoxelizerBase {
public:
    VoxelizerCPU() : VoxelizerBase(false) {}
    virtual void voxelize(VoxelGrid& grid) = 0;
    VoxelGrid voxelize(float resolution,
                      const Eigen::Vector3f& min_bounds,
                      const Eigen::Vector3f& max_bounds) override;
};
```

### VoxelizerGPU

GPU 实现的基类。

```cpp
class VoxelizerGPU : public VoxelizerBase {
public:
    VoxelizerGPU() : VoxelizerBase(true) {}
    virtual void voxelize(VoxelGrid& grid) = 0;
    VoxelGrid voxelize(float resolution,
                      const Eigen::Vector3f& min_bounds,
                      const Eigen::Vector3f& max_bounds) override;
};
```

## 几何体素化器

### BoxVoxelizer

体素化 3D 盒子。

```cpp
class BoxVoxelizerCPU : public VoxelizerCPU {
public:
    BoxVoxelizerCPU(const Eigen::Vector3f& center, const Eigen::Vector3f& size);
    void voxelize(VoxelGrid& grid) override;
private:
    Eigen::Vector3f center_;
    Eigen::Vector3f size_;
};
```

### SphereVoxelizer

体素化 3D 球体。

```cpp
class SphereVoxelizerCPU : public VoxelizerCPU {
public:
    SphereVoxelizerCPU(const Eigen::Vector3f& center, float radius);
    void voxelize(VoxelGrid& grid) override;
private:
    Eigen::Vector3f center_;
    float radius_;
};
```

### LineVoxelizer

使用多种算法体素化 3D 直线。

```cpp
class LineVoxelizerCPU : public VoxelizerCPU {
public:
    enum class Algorithm {RLV, SLV, ILV, BRESENHAM};
    LineVoxelizerCPU(const Eigen::Vector3f& start,
                    const Eigen::Vector3f& end,
                    Algorithm algorithm = Algorithm::BRESENHAM);
    void voxelize(VoxelGrid& grid) override;
private:
    Eigen::Vector3f start_;
    Eigen::Vector3f end_;
    Algorithm algorithm_;
};
```

### PolylineVoxelizer

体素化 3D 折线。

```cpp
class PolylineVoxelizerCPU : public VoxelizerCPU {
public:
    PolylineVoxelizerCPU(const std::vector<Eigen::Vector3f>& points,
                        LineVoxelizerCPU::Algorithm algorithm = LineVoxelizerCPU::Algorithm::BRESENHAM);
    void voxelize(VoxelGrid& grid) override;
private:
    std::vector<Eigen::Vector3f> points_;
    LineVoxelizerCPU::Algorithm algorithm_;
};
```

### SplineVoxelizer

#### 工厂类

```cpp
class SplineVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(
        const std::vector<Eigen::Vector3f>& control_points,
        float radius,
        int spline_type = 0,
        bool use_gpu = false
    );
};
```

创建一个样条曲线体素化器，可以沿样条曲线生成管状结构。

** 参数：**
- `control_points`：定义样条曲线的控制点（至少需要 4 个点）
- `radius`：样条曲线周围的管状半径
- `spline_type`：使用的样条类型：
  - 0: Catmull-Rom 样条（默认）
  - 1: B 样条
  - 2: 贝塞尔曲线
- `use_gpu`：是否使用 GPU 实现

** 可能抛出的异常：**
- `std::invalid_argument` 如果：
  - 控制点数量少于 4 个
  - 半径不是正数
  - 样条类型无效

#### CPU 实现

```cpp
class SplineVoxelizerCPU : public VoxelizerCPU {
public:
    SplineVoxelizerCPU(
        const std::vector<Eigen::Vector3f>& control_points,
        float radius,
        int spline_type
    );
    void voxelize(VoxelGrid& grid) override;
};
```

样条曲线体素化器的 CPU 实现。

** 特性：**
- 支持三种样条类型：Catmull-Rom、B 样条和贝塞尔曲线
- 高效的边界框计算，限制体素化区域
- 自适应采样样条段，实现精确的距离计算
- 线程安全的实现

#### GPU 实现

```cpp
class SplineVoxelizerGPU : public VoxelizerGPU {
public:
    SplineVoxelizerGPU(
        const std::vector<Eigen::Vector3f>& control_points,
        float radius,
        int spline_type
    );
    void voxelize(VoxelGrid& grid) override;
};
```

使用 CUDA 的样条曲线体素化器 GPU 实现。

** 特性：**
- 使用 CUDA 并行处理体素网格
- 高效的设备内存管理
- 优化的内核启动配置
- CUDA 操作的错误处理

** 性能考虑：**
- 使用 8x8x8 线程块实现最佳占用率
- 最小化 CPU-GPU 数据传输
- 高效的边界框计算，限制内核执行区域
- 自适应采样样条段

** 使用示例：**

```cpp
// 创建样条曲线体素化器
std::vector<Eigen::Vector3f> control_points = {
    {0, 0, 0},
    {1, 1, 0},
    {2, 0, 1},
    {3, 1, 1}
};
auto voxelizer = SplineVoxelizer::create(control_points, 0.1f, 0, true);

// 创建体素网格
VoxelGrid grid({100, 100, 100}, {0, 0, 0}, {10, 10, 10});

// 体素化样条曲线
voxelizer->voxelize(grid);
```

## VoxelGrid

管理体素数据的核心类。

```cpp
class VoxelGrid {
public:
    // 构造函数
    VoxelGrid(float resolution,
             const Eigen::Vector3f& min_bounds,
             const Eigen::Vector3f& max_bounds,
             bool use_gpu = false);

    // 访问器
    float resolution() const;
    const Eigen::Vector3f& min_bounds() const;
    const Eigen::Vector3f& max_bounds() const;
    const Eigen::Vector3i& dimensions() const;
    bool use_gpu() const;

    // 网格操作
    bool get(const Eigen::Vector3i& position) const;
    void set(const Eigen::Vector3i& position, bool value);
    void fill(bool value = true);
    void clear();
    void set_region(const Eigen::Vector3i& min, const Eigen::Vector3i& max, bool value);

    // 坐标转换
    Eigen::Vector3i world_to_grid(const Eigen::Vector3f& world_pos) const;
    Eigen::Vector3f grid_to_world(const Eigen::Vector3i& grid_pos) const;

    // GPU 操作
    void upload_to_gpu();
    void download_from_gpu();
    bool* device_data();
    const bool* device_data() const;

    // 统计信息
    size_t count_occupied() const;
    float occupancy_rate() const;

private:
    float resolution_;
    Eigen::Vector3f min_bounds_;
    Eigen::Vector3f max_bounds_;
    Eigen::Vector3i dimensions_;
    bool use_gpu_;
    std::vector<bool> data_;
    bool* d_data_;
};
```

## 工厂类

每个几何体素化器都有一个对应的工厂类用于创建实例。

### BoxVoxelizer 工厂

```cpp
class BoxVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(
        const Eigen::Vector3f& center,
        const Eigen::Vector3f& size,
        bool use_gpu = false
    );
};
```

### SphereVoxelizer 工厂

```cpp
class SphereVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(
        const Eigen::Vector3f& center,
        float radius,
        bool use_gpu = false
    );
};
```

### LineVoxelizer 工厂

```cpp
class LineVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(
        const Eigen::Vector3f& start,
        const Eigen::Vector3f& end,
        bool use_gpu = false,
        LineVoxelizerCPU::Algorithm algorithm = LineVoxelizerCPU::Algorithm::BRESENHAM
    );
};
```

### PolylineVoxelizer 工厂

```cpp
class PolylineVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(
        const std::vector<Eigen::Vector3f>& points,
        bool use_gpu = false,
        LineVoxelizerCPU::Algorithm algorithm = LineVoxelizerCPU::Algorithm::BRESENHAM
    );
};
```

## 错误处理

所有类都使用异常进行错误处理：

- `std::invalid_argument`：无效参数
- `std::out_of_range`：越界访问
- `std::runtime_error`：CUDA 错误和其他运行时错误

## 内存管理

- CPU 内存通过 `std::vector` 自动管理
- GPU 内存通过 CUDA 函数管理
- 拷贝构造函数和赋值运算符处理深拷贝
- 移动构造函数和赋值运算符处理资源转移