#pragma once

#include "binary_operator.hpp"

namespace VXZ {

/**
 * @brief Union operator for VoxelGrid objects
 * 
 * This operator performs a logical OR operation between two VoxelGrid objects.
 * The result is a new VoxelGrid where a voxel is occupied if it is occupied
 * in either of the input grids.
 */
class UnionOperator : public BinaryOperator {
public:
    UnionOperator() = default;
    ~UnionOperator() override = default;

    std::unique_ptr<VoxelGrid> apply(const VoxelGrid& grid1, const VoxelGrid& grid2) override;
};

} // namespace VXZ 