#pragma once
#include "voxelizer/voxelizer_base.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>
#include <unordered_map>

namespace VXZ {

class SurfaceVoxelizer : public VoxelizerBase {
public:
    // 体素化质量配置
    struct VoxelizationConfig {
        float surface_threshold = 0.01f;    // 表面采样精度阈值
        float normal_weight = 0.5f;         // 法向量权重
        int min_samples_per_voxel = 5;      // 每个体素最小采样点数
        bool use_adaptive_sampling = true;   // 是否使用自适应采样
        bool preserve_sharp_features = true; // 是否保留尖锐特征
    };

    bool voxelize_rasterization(VoxelGrid &grid) const;
    bool voxelize_slicing(VoxelGrid &grid) const;
    SurfaceVoxelizer(const VoxelizationConfig &config);
    ~SurfaceVoxelizer() override = default;

    // 设置输入表面数据
    void set_surface(const std::vector<Eigen::Vector3f>& vertices,
                    const std::vector<Eigen::Vector3f>& normals,
                    const std::vector<Eigen::Vector3i>& faces);
    // 主体素化接口
    bool voxelize(VoxelGrid& grid) const;

private:
    // 表面采样和分析
    struct SurfacePoint {
        Eigen::Vector3f position;
        Eigen::Vector3f normal;
        float curvature;
    };

    // 辅助函数
    void compute_surface_properties();
    float compute_local_feature_size(const Eigen::Vector3f& point) const;
    bool is_sharp_feature(const Eigen::Vector3f& point, const Eigen::Vector3f& normal) const;
    std::vector<SurfacePoint> generate_surface_samples() const;
    void adaptive_surface_sampling(std::vector<SurfacePoint>& samples) const;
    float signed_distance_to_surface(const Eigen::Vector3f& point) const;

    // 八叉树加速结构
    struct OctreeNode {
        std::vector<size_t> point_indices;
        std::array<std::shared_ptr<OctreeNode>, 8> children;
        Eigen::AlignedBox3f bbox;
    };
    std::shared_ptr<OctreeNode> build_octree(const std::vector<SurfacePoint>& points) const;
    void query_nearest_points(const Eigen::Vector3f& query,
                            const std::shared_ptr<OctreeNode>& node,
                            std::vector<size_t>& result) const;

    // 成员变量
    VoxelizationConfig config_;
    std::vector<Eigen::Vector3f> vertices_;
    std::vector<Eigen::Vector3f> normals_;
    std::vector<Eigen::Vector3i> faces_;
    std::vector<float> vertex_curvatures_;
    std::shared_ptr<OctreeNode> octree_root_;
};

} // namespace VXZ