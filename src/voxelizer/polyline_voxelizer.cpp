#include "voxelizer/polyline_voxelizer.hpp"
#include "voxelizer/line_voxelizer.hpp"
#include <stdexcept>

namespace VXZ {

PolylineVoxelizerCPU::PolylineVoxelizerCPU(const std::vector<Eigen::Vector3f>& points,
                                         LineVoxelizerCPU::Algorithm algorithm)
    : points_(points), algorithm_(algorithm) {
    if (points.size() < 2) {
        throw std::invalid_argument("Polyline must have at least 2 points");
    }
}

void PolylineVoxelizerCPU::voxelize_cpu(VoxelGrid& grid) {
    // Voxelize each line segment of the polyline
    for (size_t i = 0; i < points_.size() - 1; ++i) {
        // Create a line voxelizer for this segment
        auto line_voxelizer = LineVoxelizer::create(
            points_[i],
            points_[i + 1],
            false,  // use CPU
            algorithm_
        );
        
        // Voxelize the line segment
        line_voxelizer->voxelize(grid);
    }
}

PolylineVoxelizerGPU::PolylineVoxelizerGPU(const std::vector<Eigen::Vector3f>& points,
                                         LineVoxelizerCPU::Algorithm algorithm)
    : points_(points), algorithm_(algorithm) {
    if (points.size() < 2) {
        throw std::invalid_argument("Polyline must have at least 2 points");
    }
}

void PolylineVoxelizerGPU::voxelize_gpu(VoxelGrid& grid) {
    // Voxelize each line segment of the polyline
    for (size_t i = 0; i < points_.size() - 1; ++i) {
        // Create a line voxelizer for this segment
        auto line_voxelizer = LineVoxelizer::create(
            points_[i],
            points_[i + 1],
            true,  // use GPU
            algorithm_
        );
        
        // Voxelize the line segment
        line_voxelizer->voxelize(grid);
    }
}

std::unique_ptr<VoxelizerBase> PolylineVoxelizer::create(
    const std::vector<Eigen::Vector3f>& points,
    bool use_gpu,
    LineVoxelizerCPU::Algorithm algorithm) {
    if (use_gpu) {
        return std::make_unique<PolylineVoxelizerGPU>(points, algorithm);
    } else {
        return std::make_unique<PolylineVoxelizerCPU>(points, algorithm);
    }
}

} // namespace voxelization 