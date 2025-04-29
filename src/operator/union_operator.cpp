#include "operator/union_operator.hpp"
#include "core/voxel_grid.hpp"

std::unique_ptr<VXZ::VoxelGrid> VXZ::UnionOperator::apply(const VXZ::VoxelGrid &grid1, const VXZ::VoxelGrid &grid2)
{
    if (grid1.dimensions() != grid2.dimensions()) {
        throw std::invalid_argument("Grid dimensions must match");
    }

    auto result = std::make_unique<VXZ::VoxelGrid>(grid1.dimensions());

    for (int x = 0; x < grid1.dimensions().x(); x++) {
        for (int y = 0; y < grid1.dimensions().y(); y++) {
            for (int z = 0; z < grid1.dimensions().z(); z++) {
                result->set(x, y, z, grid1.get(x, y, z) || grid2.get(x, y, z));
            }
        }
    }

    return result;
}