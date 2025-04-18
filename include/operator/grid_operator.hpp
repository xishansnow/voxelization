#pragma once

#include <memory>
#include <eigen3/Eigen/Dense>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range3d.h>
#include "../core/voxel_grid.hpp"

namespace VXZ {

/**
 * @brief Base class for all grid operations
 */
class GridOperator {
public:
    virtual ~GridOperator() = default;

    /**
     * @brief Apply the operation to a voxel grid
     * @param grid The voxel grid to operate on
     * @return true if successful, false otherwise
     */
    virtual bool apply(VXZ::VoxelGrid& grid) const = 0;

protected:
    /**
     * @brief Apply operation in parallel over a 3D range
     * @tparam Func Type of the function to apply
     * @param grid The voxel grid to operate on
     * @param func The function to apply to each voxel
     */
    template<typename Func>
    void parallel_apply(VXZ::VoxelGrid& grid, Func func) const {
        tbb::parallel_for(
            tbb::blocked_range3d<int>(0, grid.get_depth(), 0, grid.get_height(), 0, grid.get_width()),
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

/**
 * @brief Smooth operation for voxel grids
 */
class SmoothOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param iterations Number of smoothing iterations
     * @param threshold Threshold for smoothing
     */
    explicit SmoothOperator(int iterations = 1, float threshold = 0.5f);

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    int iterations_;
    float threshold_;
};

/**
 * @brief Dilate operation for voxel grids
 */
class DilateOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param iterations Number of dilation iterations
     */
    explicit DilateOperator(int iterations = 1);

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    int iterations_;
};

/**
 * @brief Erode operation for voxel grids
 */
class ErodeOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param iterations Number of erosion iterations
     */
    explicit ErodeOperator(int iterations = 1);

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    int iterations_;
};

/**
 * @brief Offset operation for voxel grids
 */
class OffsetOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param distance Offset distance
     */
    explicit OffsetOperator(float distance = 1.0f);

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    float distance_;
};

/**
 * @brief Boolean union operation for voxel grids
 */
class UnionOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param other The other grid to union with
     */
    explicit UnionOperator(const VXZ::VoxelGrid& other);

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    VXZ::VoxelGrid other_;
};

/**
 * @brief Boolean intersection operation for voxel grids
 */
class IntersectionOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param other The other grid to intersect with
     */
    explicit IntersectionOperator(const VXZ::VoxelGrid& other);

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    VXZ::VoxelGrid other_;
};

/**
 * @brief Boolean difference operation for voxel grids
 */
class DifferenceOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param other The other grid to subtract
     */
    explicit DifferenceOperator(const VXZ::VoxelGrid& other);

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    VXZ::VoxelGrid other_;
};

/**
 * @brief Opening operation for voxel grids (erosion followed by dilation)
 */
class OpeningOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param iterations Number of iterations
     */
    explicit OpeningOperator(int iterations = 1);

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    int iterations_;
};

/**
 * @brief Closing operation for voxel grids (dilation followed by erosion)
 */
class ClosingOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param iterations Number of iterations
     */
    explicit ClosingOperator(int iterations = 1);

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    int iterations_;
};

/**
 * @brief Distance transform operation for voxel grids
 */
class DistanceTransformOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param max_distance Maximum distance to compute
     */
    explicit DistanceTransformOperator(float max_distance = std::numeric_limits<float>::max());

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    float max_distance_;
};

/**
 * @brief Connected components labeling operation for voxel grids
 */
class ConnectedComponentsOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param connectivity Connectivity type (6 or 26)
     */
    explicit ConnectedComponentsOperator(int connectivity = 6);

    bool apply(VXZ::VoxelGrid& grid) const override;

    /**
     * @brief Get the number of connected components
     * @return Number of connected components
     */
    int get_component_count() const;

private:
    int connectivity_;
    int component_count_;
};

/**
 * @brief Fill operation for voxel grids
 */
class FillOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param seed_point Seed point for filling
     * @param connectivity Connectivity type (6 or 26)
     */
    explicit FillOperator(
        const Eigen::Vector3i& seed_point,
        int connectivity = 6
    );

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    Eigen::Vector3i seed_point_;
    int connectivity_;
};

/**
 * @brief Interpolation operation for voxel grids
 */
class InterpolationOperator : public GridOperator {
public:
    /**
     * @brief Constructor
     * @param position The position to interpolate at
     */
    explicit InterpolationOperator(const Eigen::Vector3f& position);

    /**
     * @brief Get the interpolated value at the specified position
     * @return The interpolated value
     */
    float get_value() const;

    bool apply(VXZ::VoxelGrid& grid) const override;

private:
    Eigen::Vector3f position_;
    float interpolated_value_;
};

} // namespace VXZ 