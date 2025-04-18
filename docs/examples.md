# Usage Examples

This document provides practical examples of using the Voxelization library.

## Basic Usage

### Creating a Voxel Grid

```cpp
#include "core/voxel_grid.hpp"

// Create a voxel grid with 0.1m resolution
VoxelGrid grid(0.1f,                    // resolution
              Eigen::Vector3f(-1, -1, -1),  // min bounds
              Eigen::Vector3f(1, 1, 1),     // max bounds
              false);                    // use CPU
```

### Voxelizing a Box

```cpp
#include "voxelizer/box_voxelizer.hpp"

// Create a box voxelizer
auto box_voxelizer = BoxVoxelizer::create(
    Eigen::Vector3f(0, 0, 0),  // center
    Eigen::Vector3f(1, 1, 1),  // size
    false                      // use CPU
);

// Voxelize the box
box_voxelizer->voxelize(grid);
```

### Voxelizing a Sphere

```cpp
#include "voxelizer/sphere_voxelizer.hpp"

// Create a sphere voxelizer
auto sphere_voxelizer = SphereVoxelizer::create(
    Eigen::Vector3f(0, 0, 0),  // center
    1.0f,                      // radius
    false                      // use CPU
);

// Voxelize the sphere
sphere_voxelizer->voxelize(grid);
```

## Advanced Usage

### Voxelizing a Line

```cpp
#include "voxelizer/line_voxelizer.hpp"

// Create a line voxelizer using Bresenham algorithm
auto line_voxelizer = LineVoxelizer::create(
    Eigen::Vector3f(0, 0, 0),  // start
    Eigen::Vector3f(1, 1, 1),  // end
    false,                     // use CPU
    LineVoxelizerCPU::Algorithm::BRESENHAM
);

// Voxelize the line
line_voxelizer->voxelize(grid);
```

### Voxelizing a Polyline

```cpp
#include "voxelizer/polyline_voxelizer.hpp"

// Create a polyline
std::vector<Eigen::Vector3f> points = {
    Eigen::Vector3f(0, 0, 0),
    Eigen::Vector3f(1, 0, 0),
    Eigen::Vector3f(1, 1, 0),
    Eigen::Vector3f(0, 1, 0)
};

// Create a polyline voxelizer
auto polyline_voxelizer = PolylineVoxelizer::create(
    points,                    // points
    false,                     // use CPU
    LineVoxelizerCPU::Algorithm::BRESENHAM
);

// Voxelize the polyline
polyline_voxelizer->voxelize(grid);
```

## GPU Usage

### Creating a GPU Voxel Grid

```cpp
// Create a voxel grid with GPU support
VoxelGrid grid(0.1f,                    // resolution
              Eigen::Vector3f(-1, -1, -1),  // min bounds
              Eigen::Vector3f(1, 1, 1),     // max bounds
              true);                     // use GPU
```

### Voxelizing with GPU

```cpp
// Create a box voxelizer with GPU support
auto box_voxelizer = BoxVoxelizer::create(
    Eigen::Vector3f(0, 0, 0),  // center
    Eigen::Vector3f(1, 1, 1),  // size
    true                       // use GPU
);

// Voxelize the box using GPU
box_voxelizer->voxelize(grid);
```

## Grid Operations

### Accessing Voxels

```cpp
// Get a voxel value
Eigen::Vector3i position(10, 10, 10);
bool value = grid.get(position);

// Set a voxel value
grid.set(position, true);
```

### Batch Operations

```cpp
// Fill the entire grid
grid.fill(true);

// Clear the entire grid
grid.clear();

// Set a region of the grid
Eigen::Vector3i min(0, 0, 0);
Eigen::Vector3i max(10, 10, 10);
grid.set_region(min, max, true);
```

### Coordinate Conversion

```cpp
// Convert world coordinates to grid coordinates
Eigen::Vector3f world_pos(0.5f, 0.5f, 0.5f);
Eigen::Vector3i grid_pos = grid.world_to_grid(world_pos);

// Convert grid coordinates to world coordinates
Eigen::Vector3f world_pos2 = grid.grid_to_world(grid_pos);
```

### Statistics

```cpp
// Count occupied voxels
size_t occupied = grid.count_occupied();

// Calculate occupancy rate
float rate = grid.occupancy_rate();
```

## Error Handling

```cpp
try {
    // Create a voxel grid
    VoxelGrid grid(0.1f,
                  Eigen::Vector3f(-1, -1, -1),
                  Eigen::Vector3f(1, 1, 1),
                  true);
    
    // Voxelize a box
    auto box_voxelizer = BoxVoxelizer::create(
        Eigen::Vector3f(0, 0, 0),
        Eigen::Vector3f(1, 1, 1),
        true
    );
    box_voxelizer->voxelize(grid);
    
} catch (const std::invalid_argument& e) {
    // Handle invalid arguments
    std::cerr << "Invalid argument: " << e.what() << std::endl;
} catch (const std::out_of_range& e) {
    // Handle out of range errors
    std::cerr << "Out of range: " << e.what() << std::endl;
} catch (const std::runtime_error& e) {
    // Handle runtime errors (including CUDA errors)
    std::cerr << "Runtime error: " << e.what() << std::endl;
}
```

## Performance Tips

1. Use GPU for large grids
2. Minimize CPU-GPU data transfers
3. Use batch operations when possible
4. Choose appropriate resolution
5. Reuse voxel grids when possible

### Spline Voxelization

#### Creating a Spline Voxelizer

```cpp
// Create control points for the spline
std::vector<Eigen::Vector3f> control_points = {
    {0, 0, 0},
    {1, 1, 0},
    {2, 0, 1},
    {3, 1, 1}
};

// Create a spline voxelizer with Catmull-Rom spline
auto voxelizer = SplineVoxelizer::create(control_points, 0.1f, 0, false);

// Create a voxel grid
VoxelGrid grid({100, 100, 100}, {0, 0, 0}, {10, 10, 10});

// Voxelize the spline
voxelizer->voxelize(grid);
```

#### Using Different Spline Types

```cpp
// Create a B-spline voxelizer
auto bspline_voxelizer = SplineVoxelizer::create(control_points, 0.1f, 1, false);

// Create a Bezier curve voxelizer
auto bezier_voxelizer = SplineVoxelizer::create(control_points, 0.1f, 2, false);
```

#### GPU Acceleration

```cpp
// Create a GPU-accelerated spline voxelizer
auto gpu_voxelizer = SplineVoxelizer::create(control_points, 0.1f, 0, true);

// Create a large voxel grid
VoxelGrid large_grid({500, 500, 500}, {0, 0, 0}, {10, 10, 10});

// Voxelize using GPU
gpu_voxelizer->voxelize(large_grid);
```

#### Error Handling

```cpp
try {
    // Attempt to create a spline voxelizer with invalid parameters
    std::vector<Eigen::Vector3f> invalid_points = {{0, 0, 0}};  // Too few points
    auto voxelizer = SplineVoxelizer::create(invalid_points, 0.1f, 0, false);
} catch (const std::invalid_argument& e) {
    std::cerr << "Error: " << e.what() << std::endl;
} 