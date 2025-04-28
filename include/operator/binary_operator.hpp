#pragma once

#include <core/voxel_grid.hpp>
#include <memory>

namespace VXZ {

/**
 * @brief Base class for binary operators that operate on two VoxelGrid objects
 * 
 * This class defines the interface for binary operators that take two VoxelGrid objects
 * as input and produce a new VoxelGrid as output. Derived classes should implement
 * the specific operation logic in the apply() method.
 */
class BinaryOperator {
public:
    BinaryOperator() = default;
    virtual ~BinaryOperator() = default;

    /**
     * @brief Apply the binary operation to two VoxelGrid objects
     * 
     * @param grid1 First input VoxelGrid
     * @param grid2 Second input VoxelGrid
     * @return std::unique_ptr<VoxelGrid> Result of the operation
     */
    virtual std::unique_ptr<VoxelGrid> apply(const VoxelGrid& grid1, const VoxelGrid& grid2) = 0;

protected:
    /**
     * @brief Create a new VoxelGrid with the same parameters as the input grids
     * 
     * @param grid1 First input grid (used for parameters)
     * @return std::unique_ptr<VoxelGrid> New grid with same parameters
     */
    std::unique_ptr<VoxelGrid> createResultGrid(const VoxelGrid& grid1) const {
        return std::make_unique<VoxelGrid>(
            grid1.resolution(),
            grid1.min_bounds(),
            grid1.max_bounds()
        );
    }

    /**
     * @brief Check if two grids have compatible dimensions
     * 
     * @param grid1 First grid
     * @param grid2 Second grid
     * @return true if grids are compatible
     * @return false if grids are not compatible
     */
    bool areGridsCompatible(const VoxelGrid& grid1, const VoxelGrid& grid2) const {
        return grid1.resolution() == grid2.resolution() &&
               grid1.min_bounds() == grid2.min_bounds() &&
               grid1.max_bounds() == grid2.max_bounds() &&
               grid1.dimensions() == grid2.dimensions();
    }
};

} // namespace VXZ 