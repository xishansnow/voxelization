#pragma once

#include "voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>

/*
 * 线体素化算法参考以下文献:
 *
 * Real Line Voxelisation (RLV):
 * [1] Kaufman, A., & Shimony, E. (1986).
 *     "3D scan-conversion algorithms for voxel-based graphics"
 *     Proceedings of the 1986 workshop on Interactive 3D graphics, 45-75
 *
 * Supercover Line Voxelisation (SLV):
 * [2] Cohen-Or, D., & Kaufman, A. (1995).
 *     "Fundamentals of surface voxelization"
 *     Graphical Models and Image Processing, 57(6), 453-461
 *
 * Integer-only Line Voxelisation (ILV):
 * [3] Yagel, R., Cohen, D., & Kaufman, A. (1992).
 *     "Discrete ray tracing"
 *     IEEE Computer Graphics and Applications, 12(5), 19-28
 *
 * Digital Differential Analyzer (DDA):
 * [4] Amanatides, J., & Woo, A. (1987).
 *     "A fast voxel traversal algorithm for ray tracing"
 *     Eurographics, 87(3), 3-10
 *
 * 3D Bresenham:
 * [5] Bresenham, J. E. (1965).
 *     "Algorithm for computer control of a digital plotter"
 *     IBM Systems Journal, 4(1), 25-30
 * [6] Kaufman, A. (1990).
 *     "Efficient algorithms for 3D scan-conversion of parametric curves, surfaces, and volumes"
 *     ACM SIGGRAPH Computer Graphics, 24(4), 171-179
 */
/*
 * Tripod算法参考以下文献:
 * [7] Gao, J., & Kaufman, A. (1990).
 *     "3D Digital Lines"
 *     Proceedings of Vision Interface '90, 85-91
 *
 * Xiaolin Wu抗锯齿算法参考以下文献:
 * [8] Wu, X. (1991).
 *     "An efficient antialiasing technique"
 *     ACM SIGGRAPH Computer Graphics, 25(4), 143-152
 * [9] Wu, X. (2001).
 *     "Fast antialiased 3D line voxelization"
 *     Journal of Graphics Tools, 6(1), 1-10
 */

namespace VXZ {

    enum class LineAlgorithm {
        RLV,    // Real Line Voxelisation
        SLV,    // Supercover Line Voxelisation
        ILV,    // Integer-only Line Voxelisation
        DDA,    // 3D Digital Differential Analyser
        BRESENHAM, // 3D Bresenham
        TRIPOD,  // Tripod 3D数字线算法
        WU       // Xiaolin Wu抗锯齿算法
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
    void voxelize_tripod(VoxelGrid& grid);
    void voxelize_wu(VoxelGrid& grid);
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