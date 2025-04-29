#include "voxelizer/LevelSetVoxelizer.hpp"

    /* Level Set 体素化算法实现参考以下文献:
     *
     * [1] Osher, S., & Fedkiw, R. (2003). Level set methods and dynamic implicit surfaces. 
     *     Springer Science & Business Media.
     * 
     * [2] Sethian, J. A. (1999). Level set methods and fast marching methods. 
     *     Cambridge university press.
     *
     * 本实现采用直接采样法,通过评估每个网格点的 level set 函数值来确定其内外性。
     * 对于更高效的实现,可以考虑:
     * 1. 窄带(Narrow Band)方法 - 只在零等值面附近进行采样
     * 2. 自适应采样 - 在曲率大的区域进行更密集的采样
     * 3. 快速行进(Fast Marching)方法 - 用于重建距离场
     */


namespace VXZ {

void LevelSetVoxelizerCPU::voxelize(VoxelGrid& grid) {
    // 计算网格参数
    const float resolution = grid.resolution();
    const Eigen::Vector3f min_bounds = grid.min_bounds();
    const Eigen::Vector3f max_bounds = grid.max_bounds();
    
    // 计算网格尺寸
    const int nx = grid.get_size_x();
    const int ny = grid.get_size_y(); 
    const int nz = grid.get_size_z();

    // 初始化窄带参数
    const float narrow_band_width = 3.0f * resolution; // 窄带宽度
    
    // 使用窄带方法进行采样
    #pragma omp parallel for collapse(3)
    for (int z = 0; z < nz; ++z) {
        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                // 计算世界坐标
                Eigen::Vector3f pos = min_bounds + Eigen::Vector3f(
                    x * resolution,
                    y * resolution, 
                    z * resolution
                );
                
                // 计算level set值
                float phi = level_set_function(pos);
                
                // 仅在窄带区域内进行精确计算
                if (std::abs(phi) <= narrow_band_width) {
                    // 计算梯度
                    const float eps = resolution * 0.1f;
                    Eigen::Vector3f grad(
                        (level_set_function(pos + Eigen::Vector3f(eps,0,0)) - 
                         level_set_function(pos - Eigen::Vector3f(eps,0,0))) / (2*eps),
                        (level_set_function(pos + Eigen::Vector3f(0,eps,0)) - 
                         level_set_function(pos - Eigen::Vector3f(0,eps,0))) / (2*eps),
                        (level_set_function(pos + Eigen::Vector3f(0,0,eps)) - 
                         level_set_function(pos - Eigen::Vector3f(0,0,eps))) / (2*eps)
                    );
                    
                    // 计算曲率
                    float curvature = grad.norm();
                    
                    // 根据曲率自适应调整采样
                    if (curvature > 1.0f) {
                        // 在高曲率区域进行更精细的采样
                        const int refine = 2;
                        bool is_inside = false;
                        for (int rz = 0; rz < refine; ++rz) {
                            for (int ry = 0; ry < refine; ++ry) {
                                for (int rx = 0; rx < refine; ++rx) {
                                    Eigen::Vector3f refined_pos = pos + Eigen::Vector3f(
                                        (rx - refine/2) * resolution/refine,
                                        (ry - refine/2) * resolution/refine,
                                        (rz - refine/2) * resolution/refine
                                    );
                                    if (level_set_function(refined_pos) <= 0) {
                                        is_inside = true;
                                        break;
                                    }
                                }
                            }
                        }
                        grid.set(x, y, z, is_inside);
                    } else {
                        // 在低曲率区域直接使用level set值
                        grid.set(x, y, z, phi <= 0);
                    }
                } else {
                    // 在窄带外直接使用符号
                    grid.set(x, y, z, phi <= 0);
                }
            }
        }
    }
}

float LevelSetVoxelizerCPU::level_set_function(const Eigen::Vector3f& pos) const {
    // 默认实现:球体
    const float radius = 1.0f;
    const Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
    return (pos - center).norm() - radius;
}




void LevelSetVoxelizerGPU::voxelize(VoxelGrid &grid)
{
}
}