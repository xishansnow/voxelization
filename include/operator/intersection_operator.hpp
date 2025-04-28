#pragma once

#include "binary_operator.hpp"

namespace VXZ {

/**
 * @brief Intersection operator for VoxelGrid objects
 * 
 * This operator performs a logical AND operation between two VoxelGrid objects.
 * The result is a new VoxelGrid where a voxel is occupied only if it is occupied
 * in both input grids.
 */
class IntersectionOperator : public BinaryOperator {
public:
    IntersectionOperator() = default;
    ~IntersectionOperator() override = default;

    std::unique_ptr<VoxelGrid> apply(const VoxelGrid& grid1, const VoxelGrid& grid2) override {
        // Check if grids are compatible
        if (!areGridsCompatible(grid1, grid2)) {
            return nullptr;
        }

        // Create result grid
        auto result = createResultGrid(grid1);

        // Perform intersection operation
        for (int x = 0; x < grid1.dimensions().x(); x++) {
            for (int y = 0; y < grid1.dimensions().y(); y++) {
                for (int z = 0; z < grid1.dimensions().z(); z++) {
                    bool value = grid1.get(x, y, z) && grid2.get(x, y, z);
                    result->set(x, y, z, value);
                }
            }
        }

        return result;
    }
};

} // namespace VXZ 