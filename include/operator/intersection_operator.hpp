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

    std::unique_ptr<VoxelGrid> apply(const VoxelGrid& grid1, const VoxelGrid& grid2) override;
};

} // namespace VXZ 