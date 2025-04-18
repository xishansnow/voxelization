#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>

namespace VXZ {

// Forward declarations
class SplineVoxelizerCPU;
class SplineVoxelizerGPU;



/**
 * @brief CPU implementation of spline voxelizer
 */
class SplineVoxelizerCPU : public VoxelizerCPU {
public:
    /**
     * @brief Constructor
     * @param control_points Control points for the spline
     * @param radius Radius of the spline curve
     * @param spline_type Type of spline to use
     */
    SplineVoxelizerCPU(
        const std::vector<Eigen::Vector3f>& control_points,
        float radius,
        int spline_type
    );

    /**
     * @brief Voxelize the spline on CPU
     * @param grid Reference to the voxel grid
     */
    void voxelize(VoxelGrid& grid) override;

private:
    std::vector<Eigen::Vector3f> control_points_;
    float radius_;
    int spline_type_;

    // Spline evaluation functions
    Eigen::Vector3f evaluate_catmull_rom(float t, int segment) const;
    Eigen::Vector3f evaluate_bspline(float t, int segment) const;
    Eigen::Vector3f evaluate_bezier(float t, int segment) const;
    Eigen::Vector3f evaluate_spline(float t, int segment) const;
    Eigen::Vector3f evaluate_spline_derivative(float t, int segment) const;
    bool is_point_in_spline(const Eigen::Vector3f& point) const;
};

/**
 * @brief GPU implementation of spline voxelizer
 */
class SplineVoxelizerGPU : public VoxelizerGPU {
public:
    /**
     * @brief Constructor
     * @param control_points Control points for the spline
     * @param radius Radius of the spline curve
     * @param spline_type Type of spline to use
     */
    SplineVoxelizerGPU(
        const std::vector<Eigen::Vector3f>& control_points,
        float radius,
        int spline_type
    );

    /**
     * @brief Voxelize the spline on GPU
     * @param grid Reference to the voxel grid
     */
    void voxelize(VoxelGrid& grid) override;

private:
    std::vector<Eigen::Vector3f> control_points_;
    float radius_;
    int spline_type_;
};

} // namespace VXZ 