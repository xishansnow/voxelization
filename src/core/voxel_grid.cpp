#include "core/voxel_grid.hpp"
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <fstream>

namespace VXZ {

VoxelGrid::VoxelGrid(float resolution,
                    const Eigen::Vector3f& min_bounds,
                    const Eigen::Vector3f& max_bounds)
    : resolution_(resolution),
      min_bounds_(min_bounds),
      max_bounds_(max_bounds) {
    initialize();
}

VoxelGrid::~VoxelGrid() {
    cleanup();
}

VoxelGrid::VoxelGrid(const VoxelGrid& other)
    : resolution_(other.resolution_),
      min_bounds_(other.min_bounds_),
      max_bounds_(other.max_bounds_),
      dimensions_(other.dimensions_),
      data_(other.data_) {
}

VoxelGrid& VoxelGrid::operator=(const VoxelGrid& other) {
    if (this != &other) {
        cleanup();
        resolution_ = other.resolution_;
        min_bounds_ = other.min_bounds_;
        max_bounds_ = other.max_bounds_;
        dimensions_ = other.dimensions_;
        data_ = other.data_;
    }
    return *this;
}

VoxelGrid::VoxelGrid(VoxelGrid&& other) noexcept
    : resolution_(other.resolution_),
      min_bounds_(std::move(other.min_bounds_)),
      max_bounds_(std::move(other.max_bounds_)),
      dimensions_(std::move(other.dimensions_)),
      data_(std::move(other.data_)) {
}

VoxelGrid& VoxelGrid::operator=(VoxelGrid&& other) noexcept {
    if (this != &other) {
        cleanup();
        resolution_ = other.resolution_;
        min_bounds_ = std::move(other.min_bounds_);
        max_bounds_ = std::move(other.max_bounds_);
        dimensions_ = std::move(other.dimensions_);
        data_ = std::move(other.data_);
    }
    return *this;
}

void VoxelGrid::initialize() {
    // Calculate grid dimensions
    Eigen::Vector3f size = max_bounds_ - min_bounds_;
    dimensions_ = (size / resolution_).cast<int>() + Eigen::Vector3i::Ones();
    
    // Allocate memory
    data_.resize(dimensions_.x() * dimensions_.y() * dimensions_.z(), false);
}

void VoxelGrid::cleanup() {
    // Nothing to clean up in CPU-only version
}

bool VoxelGrid::get(const Eigen::Vector3i& position) const {
    if (!is_valid_position(position)) {
        throw std::out_of_range("Grid position out of range");
    }
    return data_[position.x() + position.y() * dimensions_.x() +
                position.z() * dimensions_.x() * dimensions_.y()];
}

void VoxelGrid::set(const Eigen::Vector3i& position, bool value) {
    if (!is_valid_position(position)) {
        throw std::out_of_range("Grid position out of range");
    }
    data_[position.x() + position.y() * dimensions_.x() +
         position.z() * dimensions_.x() * dimensions_.y()] = value;
}

Eigen::Vector3i VoxelGrid::world_to_grid(const Eigen::Vector3f& world_pos) const {
    Eigen::Vector3f relative_pos = world_pos - min_bounds_;
    return (relative_pos / resolution_).cast<int>();
}

Eigen::Vector3f VoxelGrid::grid_to_world(const Eigen::Vector3i& grid_pos) const {
    return min_bounds_ + grid_pos.cast<float>() * resolution_;
}

bool VoxelGrid::is_valid_position(const Eigen::Vector3i& position) const {
    return position.x() >= 0 && position.x() < dimensions_.x() &&
           position.y() >= 0 && position.y() < dimensions_.y() &&
           position.z() >= 0 && position.z() < dimensions_.z();
}

bool VoxelGrid::is_inside_grid(const Eigen::Vector3i& position) const {
    return is_valid_position(position);
}

void VoxelGrid::fill(bool value) {
    std::fill(data_.begin(), data_.end(), value);
}

void VoxelGrid::set_region(const Eigen::Vector3i& min, const Eigen::Vector3i& max, bool value) {
    // Validate region bounds
    if (!is_valid_position(min) || !is_valid_position(max)) {
        throw std::out_of_range("Region bounds out of range");
    }
    
    // Clamp region to grid bounds
    Eigen::Vector3i grid_min = min.cwiseMax(Eigen::Vector3i::Zero());
    Eigen::Vector3i grid_max = max.cwiseMin(dimensions_ - Eigen::Vector3i::Ones());
    
    // Set values in the region
    for (int x = grid_min.x(); x <= grid_max.x(); ++x) {
        for (int y = grid_min.y(); y <= grid_max.y(); ++y) {
            for (int z = grid_min.z(); z <= grid_max.z(); ++z) {
                set(Eigen::Vector3i(x, y, z), value);
            }
        }
    }
}

size_t VoxelGrid::count_occupied() const {
    return std::count(data_.begin(), data_.end(), true);
}

float VoxelGrid::occupancy_rate() const {
    return static_cast<float>(count_occupied()) / data_.size();
}

void VoxelGrid::save(const std::string &filename) const
{
}

VoxelGrid VoxelGrid::load(const std::string &filename)
{
    // 从文件读取体素网格参数
    std::ifstream file;

    
    file.open(filename, std::ios::binary);
    
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件: " + filename);
    }

    // 读取基本参数
    Eigen::Vector3f min_bounds;
    Eigen::Vector3f max_bounds;
    Eigen::Vector3i dimensions;
    float resolution;

    file.read(reinterpret_cast<char*>(min_bounds.data()), sizeof(min_bounds));
    file.read(reinterpret_cast<char*>(max_bounds.data()), sizeof(max_bounds));
    file.read(reinterpret_cast<char*>(dimensions.data()), sizeof(dimensions));
    file.read(reinterpret_cast<char*>(&resolution), sizeof(resolution));
   
    // 创建新的体素网格
    VoxelGrid grid(resolution, min_bounds, max_bounds);

    // 读取体素数据
    std::vector<bool> data(dimensions.prod());
    for (size_t i = 0; i < data.size(); ++i) {
        char value;
        file.read(&value, sizeof(char));
        data[i] = static_cast<bool>(value);
    }

    grid.data_ = std::move(data);
    return grid;
}
} // namespace VXZ