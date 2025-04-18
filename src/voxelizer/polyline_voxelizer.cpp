#include "voxelizer/polyline_voxelizer.hpp"
#include "voxelizer/line_voxelizer.hpp"
#include <stdexcept>

namespace VXZ {

PolylineVoxelizerCPU::PolylineVoxelizerCPU(const std::vector<Eigen::Vector3f>& points,
                                         LineAlgorithm algorithm)
    : points_(points), algorithm_(algorithm) {
    if (points.size() < 2) {
        throw std::invalid_argument("Polyline must have at least 2 points");
    }
}

void PolylineVoxelizerCPU::voxelize(VoxelGrid& grid) {
    // Voxelize each line segment of the polyline
    for (size_t i = 0; i < points_.size() - 1; ++i) {
        // Create a line voxelizer for this segment
        auto line_voxelizer = std::make_unique<LineVoxelizerCPU>(
            points_[i],
            points_[i + 1],
            algorithm_
        );
        
        // Voxelize the line segment
        line_voxelizer->voxelize(grid);
    }
}

PolylineVoxelizerGPU::PolylineVoxelizerGPU(const std::vector<Eigen::Vector3f>& points,
                                         LineAlgorithm algorithm)
    : points_(points), algorithm_(algorithm) {
    if (points.size() < 2) {
        throw std::invalid_argument("Polyline must have at least 2 points");
    }
}

void PolylineVoxelizerGPU::voxelize(VoxelGrid& grid) {
    // Voxelize each line segment of the polyline
    for (size_t i = 0; i < points_.size() - 1; ++i) {
        // Create a line voxelizer for this segment
        auto line_voxelizer = std::make_unique<LineVoxelizerGPU>(
            points_[i],
            points_[i + 1],
            algorithm_
        );
        
        // Voxelize the line segment
        line_voxelizer->voxelize(grid);
    }
}


} // namespace VXZ 