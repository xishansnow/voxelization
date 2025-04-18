# Class Reference

This document provides detailed API documentation for all classes in the Voxelization system.

## Core Classes

### VoxelizerBase

Base class for all voxelizers.

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

Base class for CPU implementations.

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

Base class for GPU implementations.

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

## Geometric Voxelizers

### BoxVoxelizer

Voxelizes a 3D box.

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

Voxelizes a 3D sphere.

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

Voxelizes a 3D line using various algorithms.

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

Voxelizes a 3D polyline.

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

## VoxelGrid

Core class for managing voxel data.

```cpp
class VoxelGrid {
public:
    // Constructors
    VoxelGrid(float resolution,
             const Eigen::Vector3f& min_bounds,
             const Eigen::Vector3f& max_bounds,
             bool use_gpu = false);

    // Accessors
    float resolution() const;
    const Eigen::Vector3f& min_bounds() const;
    const Eigen::Vector3f& max_bounds() const;
    const Eigen::Vector3i& dimensions() const;
    bool use_gpu() const;

    // Grid operations
    bool get(const Eigen::Vector3i& position) const;
    void set(const Eigen::Vector3i& position, bool value);
    void fill(bool value = true);
    void clear();
    void set_region(const Eigen::Vector3i& min, const Eigen::Vector3i& max, bool value);

    // Coordinate conversion
    Eigen::Vector3i world_to_grid(const Eigen::Vector3f& world_pos) const;
    Eigen::Vector3f grid_to_world(const Eigen::Vector3i& grid_pos) const;

    // GPU operations
    void upload_to_gpu();
    void download_from_gpu();
    bool* device_data();
    const bool* device_data() const;

    // Statistics
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

## Factory Classes

Each geometric voxelizer has a corresponding factory class for creating instances.

### BoxVoxelizer Factory

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

### SphereVoxelizer Factory

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

### LineVoxelizer Factory

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

### PolylineVoxelizer Factory

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

## Error Handling

All classes use exceptions for error handling:

- `std::invalid_argument`: Invalid parameters
- `std::out_of_range`: Out of bounds access
- `std::runtime_error`: CUDA errors and other runtime errors

## Memory Management

- CPU memory is managed automatically through `std::vector`
- GPU memory is managed through CUDA functions
- Copy constructors and assignment operators handle deep copies
- Move constructors and assignment operators handle resource transfer