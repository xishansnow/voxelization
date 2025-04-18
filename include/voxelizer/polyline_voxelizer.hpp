#pragma once

#include "voxelizer/voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>

namespace VXZ {

/**
 * @brief CPU implementation of polyline voxelization
 * 
 * This class implements the CPU version of polyline voxelization,
 * which voxelizes a 3D polyline using the specified line voxelization algorithm.
 */
class PolylineVoxelizerCPU : public VoxelizerCPU {
public:
    /**
     * @brief Construct a new Polyline Voxelizer CPU object
     * 
     * @param points Vector of points defining the polyline
     * @param algorithm Line voxelization algorithm to use
     */
    PolylineVoxelizerCPU(const std::vector<Eigen::Vector3f>& points, 
                        LineVoxelizerCPU::Algorithm algorithm = LineVoxelizerCPU::Algorithm::BRESENHAM);

    /**
     * @brief Voxelize the polyline on CPU
     * 
     * @param grid The voxel grid to fill
     */
    void voxelize_cpu(VoxelGrid& grid) override;

private:
    std::vector<Eigen::Vector3f> points_;  // Points defining the polyline
    LineVoxelizerCPU::Algorithm algorithm_;  // Line voxelization algorithm
};

/**
 * @brief GPU implementation of polyline voxelization
 * 
 * This class implements the GPU version of polyline voxelization,
 * which voxelizes a 3D polyline using CUDA.
 */
class PolylineVoxelizerGPU : public VoxelizerGPU {
public:
    /**
     * @brief Construct a new Polyline Voxelizer GPU object
     * 
     * @param points Vector of points defining the polyline
     * @param algorithm Line voxelization algorithm to use
     */
    PolylineVoxelizerGPU(const std::vector<Eigen::Vector3f>& points,
                        LineVoxelizerCPU::Algorithm algorithm = LineVoxelizerCPU::Algorithm::BRESENHAM);

    /**
     * @brief Voxelize the polyline on GPU
     * 
     * @param grid The voxel grid to fill
     */
    void voxelize_gpu(VoxelGrid& grid) override;

private:
    std::vector<Eigen::Vector3f> points_;  // Points defining the polyline
    LineVoxelizerCPU::Algorithm algorithm_;  // Line voxelization algorithm
};

/**
 * @brief Factory class for creating polyline voxelizers
 * 
 * This class provides a factory method to create either CPU or GPU
 * implementations of the polyline voxelizer.
 */
class PolylineVoxelizer {
public:
    /**
     * @brief Create a polyline voxelizer
     * 
     * @param points Vector of points defining the polyline
     * @param use_gpu Whether to use GPU implementation
     * @param algorithm Line voxelization algorithm to use
     * @return std::unique_ptr<VoxelizerBase> Pointer to the created voxelizer
     */
    static std::unique_ptr<VoxelizerBase> create(
        const std::vector<Eigen::Vector3f>& points,
        bool use_gpu = false,
        LineVoxelizerCPU::Algorithm algorithm = LineVoxelizerCPU::Algorithm::BRESENHAM
    );
};

} // namespace voxelization 