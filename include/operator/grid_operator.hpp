#pragma once

#include <memory>
#include <eigen3/Eigen/Dense>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range3d.h>
#include "../core/voxel_grid.hpp"
// #include "operator/SmoothOperator.hpp"
// #include "operator/DilateOperator.hpp"
// #include "operator/ErodeOperator.hpp"
// #include "operator/OffsetOperator.hpp"
// #include "operator/UnionOperator.hpp"
// #include "operator/IntersectionOperator.hpp"
// #include "operator/DifferenceOperator.hpp"
// #include "operator/OpeningOperator.hpp"
// #include "operator/ClosingOperator.hpp"
// #include "operator/DistanceTransformOperator.hpp"
// #include "operator/ConnectedComponentsOperator.hpp"
// #include "operator/FillOperator.hpp"
// #include "operator/InterpolationOperator.hpp"





namespace VXZ {

class GridOperator {
public:
    virtual ~GridOperator() = default;
    virtual bool apply(VXZ::VoxelGrid& grid) const = 0;

protected:
    template<typename Func>
    void parallel_apply(VXZ::VoxelGrid& grid, Func func) const {
        tbb::parallel_for(
            tbb::blocked_range3d<int>(0, grid.get_size_x(), 0, grid.get_size_y(), 0, grid.get_size_z()),
            [&](const tbb::blocked_range3d<int>& r) {
                for (int z = r.pages().begin(); z < r.pages().end(); ++z) {
                    for (int y = r.rows().begin(); y < r.rows().end(); ++y) {
                        for (int x = r.cols().begin(); x < r.cols().end(); ++x) {
                            func(x, y, z);
                        }
                    }
                }
            }
        );
    }
};

}
