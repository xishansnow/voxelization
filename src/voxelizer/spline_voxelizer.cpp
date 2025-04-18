#include "voxelizer/spline_voxelizer.hpp"
#include <stdexcept>
#include <cmath>

namespace VXZ {

// CPU implementation
SplineVoxelizerCPU::SplineVoxelizerCPU(
    const std::vector<Eigen::Vector3f>& control_points,
    float radius,
    int spline_type
) : control_points_(control_points), radius_(radius), spline_type_(spline_type) {}

Eigen::Vector3f SplineVoxelizerCPU::evaluate_catmull_rom(float t, int segment) const {
    const auto& p0 = control_points_[segment];
    const auto& p1 = control_points_[segment + 1];
    const auto& p2 = control_points_[segment + 2];
    const auto& p3 = control_points_[segment + 3];

    float t2 = t * t;
    float t3 = t2 * t;

    Eigen::Vector3f a = -0.5f * p0 + 1.5f * p1 - 1.5f * p2 + 0.5f * p3;
    Eigen::Vector3f b = p0 - 2.5f * p1 + 2.0f * p2 - 0.5f * p3;
    Eigen::Vector3f c = -0.5f * p0 + 0.5f * p2;
    Eigen::Vector3f d = p1;

    return a * t3 + b * t2 + c * t + d;
}

Eigen::Vector3f SplineVoxelizerCPU::evaluate_bspline(float t, int segment) const {
    const auto& p0 = control_points_[segment];
    const auto& p1 = control_points_[segment + 1];
    const auto& p2 = control_points_[segment + 2];
    const auto& p3 = control_points_[segment + 3];

    float t2 = t * t;
    float t3 = t2 * t;

    float b0 = (1 - t) * (1 - t) * (1 - t) / 6.0f;
    float b1 = (3 * t3 - 6 * t2 + 4) / 6.0f;
    float b2 = (-3 * t3 + 3 * t2 + 3 * t + 1) / 6.0f;
    float b3 = t3 / 6.0f;

    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

Eigen::Vector3f SplineVoxelizerCPU::evaluate_bezier(float t, int segment) const {
    const auto& p0 = control_points_[segment];
    const auto& p1 = control_points_[segment + 1];
    const auto& p2 = control_points_[segment + 2];
    const auto& p3 = control_points_[segment + 3];

    float t2 = t * t;
    float t3 = t2 * t;
    float mt = 1 - t;
    float mt2 = mt * mt;
    float mt3 = mt2 * mt;

    return p0 * mt3 + 3 * p1 * mt2 * t + 3 * p2 * mt * t2 + p3 * t3;
}

Eigen::Vector3f SplineVoxelizerCPU::evaluate_spline(float t, int segment) const {
    switch (spline_type_) {
        case 0: return evaluate_catmull_rom(t, segment);
        case 1: return evaluate_bspline(t, segment);
        case 2: return evaluate_bezier(t, segment);
        default: throw std::runtime_error("Invalid spline type");
    }
}

Eigen::Vector3f SplineVoxelizerCPU::evaluate_spline_derivative(float t, int segment) const {
    const float h = 0.001f;
    Eigen::Vector3f p1 = evaluate_spline(t - h, segment);
    Eigen::Vector3f p2 = evaluate_spline(t + h, segment);
    return (p2 - p1) / (2 * h);
}

bool SplineVoxelizerCPU::is_point_in_spline(const Eigen::Vector3f& point) const {
    const int num_segments = control_points_.size() - 3;
    const int steps_per_segment = 100;
    const float step_size = 1.0f / steps_per_segment;

    for (int segment = 0; segment < num_segments; ++segment) {
        for (int step = 0; step < steps_per_segment; ++step) {
            float t = step * step_size;
            Eigen::Vector3f spline_point = evaluate_spline(t, segment);
            Eigen::Vector3f derivative = evaluate_spline_derivative(t, segment);
            
            // Calculate distance from point to spline
            Eigen::Vector3f diff = point - spline_point;
            float distance = diff.norm();
            
            // Check if point is within radius of spline
            if (distance <= radius_) {
                return true;
            }
        }
    }
    return false;
}

void SplineVoxelizerCPU::voxelize(VoxelGrid& grid) {
    // Get grid dimensions and resolution
    const auto& dims = grid.dimensions();
    const auto& resolution = grid.resolution();
    const auto& min_bounds = grid.min_bounds();
    const auto& max_bounds = grid.max_bounds();

    // Calculate bounding box of spline
    Eigen::Vector3f min_point = control_points_[0];
    Eigen::Vector3f max_point = control_points_[0];
    for (const auto& point : control_points_) {
        min_point = min_point.cwiseMin(point);
        max_point = max_point.cwiseMax(point);
    }
    min_point.array() -= radius_;
    max_point.array() += radius_;

    // Convert to grid coordinates
    Eigen::Vector3i min_grid = grid.world_to_grid(min_point);
    Eigen::Vector3i max_grid = grid.world_to_grid(max_point);

    // Clamp to grid bounds
    min_grid = min_grid.cwiseMax(Eigen::Vector3i::Zero());
    max_grid = max_grid.cwiseMin(dims - Eigen::Vector3i::Ones());

    // Voxelize spline
    for (int z = min_grid.z(); z <= max_grid.z(); ++z) {
        for (int y = min_grid.y(); y <= max_grid.y(); ++y) {
            for (int x = min_grid.x(); x <= max_grid.x(); ++x) {
                Eigen::Vector3i grid_pos(x, y, z);
                Eigen::Vector3f world_pos = grid.grid_to_world(grid_pos);
                if (is_point_in_spline(world_pos)) {
                    grid.set(grid_pos, true);
                }
            }
        }
    }
}

// GPU implementation
SplineVoxelizerGPU::SplineVoxelizerGPU(
    const std::vector<Eigen::Vector3f>& control_points,
    float radius,
    int spline_type
) : control_points_(control_points), radius_(radius), spline_type_(spline_type) {}

void SplineVoxelizerGPU::voxelize(VoxelGrid& grid) {
    // For now, fall back to CPU implementation
    // TODO: Implement GPU version using CUDA
    voxelize(grid);
}

} // namespace VXZ 