#pragma once

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <eigen3/Eigen/Dense>
#include "../core/voxel_grid.hpp"

namespace VXZ {



/**
 * @brief Base class for all voxel storage types
 */
class VoxelStorage {
public:
    virtual ~VoxelStorage() = default;

    /**
     * @brief Save the storage to a file
     * @param filename Path to the output file
     * @return true if successful, false otherwise
     */
    virtual bool save(const std::string& filename) const = 0;

    /**
     * @brief Load the storage from a file
     * @param filename Path to the input file
     * @return true if successful, false otherwise
     */
    virtual bool load(const std::string& filename) = 0;

    /**
     * @brief Get the size of the storage in bytes
     * @return Size in bytes
     */
    virtual size_t get_size() const = 0;

    /**
     * @brief Convert the storage to a voxel grid
     * @param grid The voxel grid to fill
     * @return true if successful, false otherwise
     */
    virtual bool to_voxel_grid(VXZ::VoxelGrid& grid) const = 0;
};



} // namespace VXZ