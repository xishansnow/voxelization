#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>

namespace VXZ {

// 3D corridor voxelizer CPU implementation
class CorridorVoxelizerCPU : public VoxelizerCPU {
public:
    CorridorVoxelizerCPU(const std::vector<Eigen::Vector3f>& control_points,
                        float radius,
                        int num_segments = 100)
        : control_points_(control_points),
          radius_(radius),
          num_segments_(num_segments) {}
    
    void voxelize(VoxelGrid& grid) override;
    
private:
    std::vector<Eigen::Vector3f> control_points_;
    float radius_;
    int num_segments_;
    
    // Helper functions
    Eigen::Vector3f evaluate_cubic_spline(float t, int segment) const;
    Eigen::Vector3f evaluate_spline_derivative(float t, int segment) const;
    bool is_point_in_corridor(const Eigen::Vector3f& point) const;
};

// 3D corridor voxelizer GPU implementation
class CorridorVoxelizerGPU : public VoxelizerGPU {
public:
    CorridorVoxelizerGPU(const std::vector<Eigen::Vector3f>& control_points,
                        float radius,
                        int num_segments = 100)
        : control_points_(control_points),
          radius_(radius),
          num_segments_(num_segments) {}
    
    void voxelize(VoxelGrid& grid) override;
    
private:
    std::vector<Eigen::Vector3f> control_points_;
    float radius_;
    int num_segments_;
    

};


} // namespace VXZ 