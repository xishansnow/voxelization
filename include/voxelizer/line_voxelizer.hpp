#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>

namespace VXZ {

// 3D line voxelizer CPU implementation
class LineVoxelizerCPU : public VoxelizerCPU {
public:
    enum class Algorithm {
        RLV,    // Real Line Voxelisation
        SLV,    // Supercover Line Voxelisation
        ILV,    // Integer-only Line Voxelisation
        DDA,    // 3D Digital Differential Analyser
        BRESENHAM // 3D Bresenham
    };

    LineVoxelizerCPU(const Eigen::Vector3f& start,
                    const Eigen::Vector3f& end,
                    Algorithm algorithm = Algorithm::RLV)
        : start_(start),
          end_(end),
          algorithm_(algorithm) {}
    
    void voxelize(VoxelGrid& grid) {
        switch (algorithm_) {
            case Algorithm::RLV:
                voxelize_rlv(grid);
                break;
            case Algorithm::SLV:
                voxelize_slv(grid);
                break;
            case Algorithm::ILV:
                voxelize_ilv(grid);
                break;
            case Algorithm::DDA:
                voxelize_dda(grid);
                break;
            case Algorithm::BRESENHAM:
                voxelize_bresenham(grid);
                break;
        }
    }
    // Algorithm descriptions
    static const char* algorithm_name(Algorithm alg) {
        switch (alg) {
            case Algorithm::RLV:
                return "Real Line Voxelisation";
            case Algorithm::SLV:
                return "Supercover Line Voxelisation";
            case Algorithm::ILV:
                return "Integer-only Line Voxelisation";
            case Algorithm::DDA:
                return "3D Digital Differential Analyser";
            case Algorithm::BRESENHAM:
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
    Algorithm get_algorithm() const {
        return algorithm_;
    }
    
    // Set algorithm
    void set_algorithm(Algorithm algorithm) {
        algorithm_ = algorithm;
    }
private:
    Eigen::Vector3f start_;
    Eigen::Vector3f end_;
    Algorithm algorithm_;
    
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
    enum class Algorithm {
        RLV,    // Real Line Voxelisation
        SLV,    // Supercover Line Voxelisation
        ILV,    // Integer-only Line Voxelisation
        DDA,    // 3D Digital Differential Analyser
        BRESENHAM // 3D Bresenham
    };

    LineVoxelizerGPU(const Eigen::Vector3f& start,
                    const Eigen::Vector3f& end,
                    Algorithm algorithm = Algorithm::RLV)
        : start_(start),
          end_(end),
          algorithm_(algorithm) {}
    
    void voxelize_gpu(VoxelGrid& grid) override {
        // GPU implementation of line voxelization
        // This is a placeholder - actual GPU implementation would use CUDA or OpenCL
        // For now, we'll just call the CPU implementation
        switch (algorithm_) {
            case Algorithm::RLV:
                // Real Line Voxelisation on GPU
                break;
            case Algorithm::SLV:
                // Supercover Line Voxelisation on GPU
                break;
            case Algorithm::ILV:
                // Integer-only Line Voxelisation on GPU
                break;
            case Algorithm::DDA:
                // 3D Digital Differential Analyser on GPU
                break;
            case Algorithm::BRESENHAM:
                // 3D Bresenham on GPU
                break;
        }
    }
    
private:
    Eigen::Vector3f start_;
    Eigen::Vector3f end_;
    Algorithm algorithm_;
};

// Factory class for LineVoxelizer
class LineVoxelizer {
public:
    static std::unique_ptr<VoxelizerBase> create(const Eigen::Vector3f& start,
                                               const Eigen::Vector3f& end,
                                               bool use_gpu = false,
                                               LineVoxelizerCPU::Algorithm algorithm = LineVoxelizerCPU::Algorithm::RLV) {
        if (use_gpu) {
            return std::make_unique<LineVoxelizerGPU>(start, end, static_cast<LineVoxelizerGPU::Algorithm>(algorithm));
        } else {
            return std::make_unique<LineVoxelizerCPU>(start, end, algorithm);
        }
    }
};

} // namespace voxelization 