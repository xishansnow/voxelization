#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>
#include <string>

namespace VXZ  {

class VoxelGrid {
public:
    VoxelGrid(float resolution, 
             const Eigen::Vector3f& min_bounds,
             const Eigen::Vector3f& max_bounds);
    
    // Copy constructor
    VoxelGrid(const VoxelGrid& other);
    
    // Move constructor
    VoxelGrid(VoxelGrid&& other) noexcept;
    
    // Destructor
    ~VoxelGrid();
    
    // Assignment operators
    VoxelGrid& operator=(const VoxelGrid& other);
    VoxelGrid& operator=(VoxelGrid&& other) noexcept;
    
    // Accessors 
    float resolution() const { return resolution_; }
    const Eigen::Vector3f& min_bounds() const { return min_bounds_; }
    const Eigen::Vector3f& max_bounds() const { return max_bounds_; }
    const Eigen::Vector3i& dimensions() const { return dimensions_; }
    
    // Get the origin (minimum bounds) of the voxel grid
    const Eigen::Vector3f& origin() const { return min_bounds_; }
    
    // Grid access
    bool get(const Eigen::Vector3i& position) const;
    void set(const Eigen::Vector3i& position, bool value);

    // Get voxel value at specific coordinates
    bool get(size_t x, size_t y, size_t z) const {
        return get(Eigen::Vector3i(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)));
    }
    
    void set(size_t x, size_t y, size_t z, bool value) {
        set(Eigen::Vector3i(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)), value);
    }
    
    // Alias for backward compatibility
    bool get_voxel(size_t x, size_t y, size_t z) const {
        return get(x, y, z);
    }
     
    void set_voxel(size_t x, size_t y, size_t z, bool value) {
        set(x, y, z, value);
    }
    
    // Get grid dimensions
    size_t get_size_x() const { return static_cast<size_t>(dimensions_.x()); }
    size_t get_size_y() const { return static_cast<size_t>(dimensions_.y()); }
    size_t get_size_z() const { return static_cast<size_t>(dimensions_.z()); }

    
    // Batch operations
    void fill(bool value = true);
    void clear() { fill(false); }
    void set_region(const Eigen::Vector3i& min, const Eigen::Vector3i& max, bool value = true);
    
    // Grid validation
    bool is_valid_position(const Eigen::Vector3i& position) const;
    bool is_inside_grid(const Eigen::Vector3i& position) const;
    
    // Coordinate conversion
    Eigen::Vector3i world_to_grid(const Eigen::Vector3f& world_pos) const;
    Eigen::Vector3f grid_to_world(const Eigen::Vector3i& grid_pos) const;
    
    // Statistics
    size_t count_occupied() const;
    float occupancy_rate() const;
    
    // Save/Load
    void save(const std::string& filename) const;
    static VoxelGrid load(const std::string& filename);
    
private:
    float resolution_;
    Eigen::Vector3f min_bounds_;
    Eigen::Vector3f max_bounds_;
    Eigen::Vector3i dimensions_;
    
    // CPU data
    std::vector<bool> data_;
    
    // Helper methods
    void initialize();
    void cleanup();
};

} // namespace VXZ 