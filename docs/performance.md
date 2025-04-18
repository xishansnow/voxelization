# Performance Guide

This document provides guidelines for optimizing performance when using the Voxelization library.

## CPU vs GPU Selection

### When to Use CPU

- Small grids (less than 100x100x100 voxels)
- Simple operations with few voxels
- When GPU memory is limited
- When CUDA is not available

### When to Use GPU

- Large grids (more than 100x100x100 voxels)
- Complex operations with many voxels
- When high throughput is required
- When parallel processing is beneficial

## Memory Management

### CPU Memory

- Use stack allocation for small grids
- Use heap allocation for large grids
- Minimize memory copies
- Reuse memory when possible

### GPU Memory

- Allocate GPU memory once and reuse
- Minimize CPU-GPU transfers
- Use pinned memory for faster transfers
- Free GPU memory when not needed

## Grid Operations

### Efficient Access Patterns

```cpp
// Good: Sequential access
for (int z = 0; z < grid_size.z(); ++z) {
    for (int y = 0; y < grid_size.y(); ++y) {
        for (int x = 0; x < grid_size.x(); ++x) {
            grid.set(Eigen::Vector3i(x, y, z), value);
        }
    }
}

// Bad: Random access
for (int i = 0; i < num_points; ++i) {
    grid.set(random_positions[i], value);
}
```

### Batch Operations

```cpp
// Good: Use batch operations
grid.fill(true);
grid.set_region(min, max, true);

// Bad: Individual operations
for (int x = min.x(); x <= max.x(); ++x) {
    for (int y = min.y(); y <= max.y(); ++y) {
        for (int z = min.z(); z <= max.z(); ++z) {
            grid.set(Eigen::Vector3i(x, y, z), true);
        }
    }
}
```

## Coordinate Conversion

### Cache Conversions

```cpp
// Good: Cache conversions
Eigen::Vector3i grid_pos = grid.world_to_grid(world_pos);
bool value = grid.get(grid_pos);

// Bad: Repeated conversions
bool value = grid.get(grid.world_to_grid(world_pos));
```

### Use Grid Coordinates

```cpp
// Good: Use grid coordinates for internal operations
void process_grid(const VoxelGrid& grid) {
    Eigen::Vector3i grid_size = grid.get_size();
    for (int z = 0; z < grid_size.z(); ++z) {
        for (int y = 0; y < grid_size.y(); ++y) {
            for (int x = 0; x < grid_size.x(); ++x) {
                process_voxel(grid, Eigen::Vector3i(x, y, z));
            }
        }
    }
}

// Bad: Convert to world coordinates unnecessarily
void process_grid(const VoxelGrid& grid) {
    Eigen::Vector3i grid_size = grid.get_size();
    for (int z = 0; z < grid_size.z(); ++z) {
        for (int y = 0; y < grid_size.y(); ++y) {
            for (int x = 0; x < grid_size.x(); ++x) {
                Eigen::Vector3f world_pos = grid.grid_to_world(Eigen::Vector3i(x, y, z));
                process_voxel(grid, world_pos);
            }
        }
    }
}
```

## GPU Optimization

### Kernel Configuration

- Use appropriate block and grid sizes
- Minimize thread divergence
- Use shared memory when beneficial
- Avoid atomic operations when possible

### Memory Access

- Use coalesced memory access
- Minimize global memory access
- Use constant memory for read-only data
- Use texture memory for spatial data

## Resolution Selection

### Factors to Consider

- Required precision
- Available memory
- Performance requirements
- Application needs

### Guidelines

- Start with coarse resolution
- Increase resolution as needed
- Consider memory constraints
- Balance precision and performance

## Best Practices

1. Profile before optimizing
2. Measure performance impact
3. Consider trade-offs
4. Document optimizations
5. Test thoroughly

## Common Pitfalls

1. Unnecessary CPU-GPU transfers
2. Inefficient memory access patterns
3. Excessive coordinate conversions
4. Improper resolution selection
5. Neglecting memory management 