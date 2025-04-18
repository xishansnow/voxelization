#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>


namespace VXZ {

    enum class LineAlgorithm {
        RLV,    // Real Line Voxelisation
        SLV,    // Supercover Line Voxelisation
        ILV,    // Integer-only Line Voxelisation
        DDA,    // 3D Digital Differential Analyser
        BRESENHAM // 3D Bresenham
    };

// 3D line voxelizer CPU implementation
class LineVoxelizerCPU : public VoxelizerCPU {
public:


    LineVoxelizerCPU(const Eigen::Vector3f& start,
                    const Eigen::Vector3f& end,
                    LineAlgorithm algorithm = LineAlgorithm::RLV)
        : start_(start),
          end_(end),
          algorithm_(algorithm) {}
    
    void voxelize(VoxelGrid& grid) override;
    

    // Algorithm descriptions
    static const char* algorithm_name(LineAlgorithm alg) {
        switch (alg) {
            case LineAlgorithm::RLV:
                return "Real Line Voxelisation";
            case LineAlgorithm::SLV:
                return "Supercover Line Voxelisation";
            case LineAlgorithm::ILV:
                return "Integer-only Line Voxelisation";
            case LineAlgorithm::DDA:
                return "3D Digital Differential Analyser";
            case LineAlgorithm::BRESENHAM:
                return "3D Bresenham";
            default:
                return "Unknown Algorithm";
        }
    }
    
    // Get current algorithm name
    const char* get_algorithm_name() const {
        return algorithm_name(algorithm_);
    }
    
    // Get start point
    const Eigen::Vector3f& get_start() const {
        return start_;
    }
    
    // Get end point
    const Eigen::Vector3f& get_end() const {
        return end_;
    }
    
    // Get current algorithm
    LineAlgorithm get_algorithm() const {
        return algorithm_;
    }
    
    // Set algorithm
    void set_algorithm(LineAlgorithm algorithm) {
        algorithm_ = algorithm;
    }
private:
    Eigen::Vector3f start_;
    Eigen::Vector3f end_;
    LineAlgorithm algorithm_;
    
    // Algorithm implementations
    void voxelize_rlv(VoxelGrid& grid);
    void voxelize_slv(VoxelGrid& grid);
    void voxelize_ilv(VoxelGrid& grid);
    void voxelize_dda(VoxelGrid& grid);
    void voxelize_bresenham(VoxelGrid& grid);
};

// 3D line voxelizer GPU implementation
class LineVoxelizerGPU : public VoxelizerGPU {
public:


    LineVoxelizerGPU(const Eigen::Vector3f& start,
                    const Eigen::Vector3f& end,
                    LineAlgorithm algorithm = LineAlgorithm::RLV)
        : start_(start),
          end_(end),
          algorithm_(algorithm) {}
    
    void voxelize(VoxelGrid& grid) override {
        // GPU implementation of line voxelization
        // This is a placeholder - actual GPU implementation would use CUDA or OpenCL
        // For now, we'll just call the CPU implementation
        switch (algorithm_) {
            case LineAlgorithm::RLV:
                // Real Line Voxelisation on GPU
                break;
            case LineAlgorithm::SLV:
                // Supercover Line Voxelisation on GPU
                break;
            case LineAlgorithm::ILV:
                // Integer-only Line Voxelisation on GPU
                break;
            case LineAlgorithm::DDA:
                // 3D Digital Differential Analyser on GPU
                break;
            case LineAlgorithm::BRESENHAM:
                // 3D Bresenham on GPU
                break;
        }
    }
    
private:
    Eigen::Vector3f start_;
    Eigen::Vector3f end_;
    LineAlgorithm algorithm_;
};


} // namespace VXZ 