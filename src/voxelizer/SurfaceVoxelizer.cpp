#include "voxelizer/SurfaceVoxelizer.hpp"
#include <queue>
#include <algorithm>
#include <cmath>





namespace VXZ {

// 基于栅格化的表面体素化算法
bool SurfaceVoxelizer::voxelize_rasterization(VoxelGrid& grid) const {
    // 获取网格尺寸
    const auto grid_size = grid.dimensions();
    const auto voxel_size = grid.resolution();
    const auto grid_origin = grid.origin();

    // 遍历所有三角形面片
    for (const auto& face : faces_) {
        // 获取三角形顶点
        const auto& v0 = vertices_[face[0]];
        const auto& v1 = vertices_[face[1]]; 
        const auto& v2 = vertices_[face[2]];

        // 计算三角形包围盒
        Eigen::Vector3f bbox_min = v0.cwiseMin(v1.cwiseMin(v2));
        Eigen::Vector3f bbox_max = v0.cwiseMax(v1.cwiseMax(v2));
        
        // 转换为体素坐标
        Eigen::Vector3i min_voxel = ((bbox_min - grid_origin).array() / voxel_size).floor().cast<int>();
        Eigen::Vector3i max_voxel = ((bbox_max - grid_origin).array() / voxel_size).floor().cast<int>();

        // 限制在网格范围内
        min_voxel = min_voxel.cwiseMax(Eigen::Vector3i::Zero());
        max_voxel = max_voxel.cwiseMin(grid_size - Eigen::Vector3i::Ones());

        // 遍历包围盒内的体素
        for (int x = min_voxel.x(); x <= max_voxel.x(); ++x) {
            for (int y = min_voxel.y(); y <= max_voxel.y(); ++y) {
                for (int z = min_voxel.z(); z <= max_voxel.z(); ++z) {
                    // 计算体素中心点
                    Eigen::Vector3f voxel_pos(x, y, z);
                    Eigen::Vector3f voxel_center = grid_origin ;//+  (voxel_pos.array() + 0.5f) * voxel_size;

                    // 计算到表面的有符号距离
                    float sdf = signed_distance_to_surface(voxel_center);

                    // 根据距离场设置体素状态
                    if (std::abs(sdf) <= config_.surface_threshold) {
                        grid.set(x, y, z, true);
                    }
                }
            }
        }
    }
    return true;
}

// 基于切片的表面体素化算法
bool SurfaceVoxelizer::voxelize_slicing(VoxelGrid& grid) const {
    const auto grid_size = grid.dimensions();
    const auto voxel_size = grid.resolution();
    const auto grid_origin = grid.origin();

    // 对每个XY平面切片进行处理
    for (int z = 0; z < grid_size.z(); ++z) {
        float slice_z = grid_origin.z() + (z + 0.5f) * voxel_size;

        // 遍历所有三角形面片
        for (const auto& face : faces_) {
            const auto& v0 = vertices_[face[0]];
            const auto& v1 = vertices_[face[1]];
            const auto& v2 = vertices_[face[2]];

            // 检查三角形是否与当前切片相交
            float min_z = std::min({v0.z(), v1.z(), v2.z()});
            float max_z = std::max({v0.z(), v1.z(), v2.z()});

            if (slice_z >= min_z && slice_z <= max_z) {
                // 计算切片平面上的轮廓
                Eigen::Vector3f normal = (v1 - v0).cross(v2 - v0).normalized();
                float d = -normal.dot(v0);

                // 在XY平面上采样点
                Eigen::Vector2f bbox_min_2d(
                    std::min({v0.x(), v1.x(), v2.x()}),
                    std::min({v0.y(), v1.y(), v2.y()})
                );
                Eigen::Vector2f bbox_max_2d(
                    std::max({v0.x(), v1.x(), v2.x()}),
                    std::max({v0.y(), v1.y(), v2.y()})
                );

                // 转换为体素坐标
                Eigen::Vector2i min_voxel_2d = ((bbox_min_2d - grid_origin.head<2>()).array() / 
                    voxel_size).cast<int>();
                Eigen::Vector2i max_voxel_2d = ((bbox_max_2d - grid_origin.head<2>()).array() / 
                    voxel_size).cast<int>();

                // 限制在网格范围内
                min_voxel_2d = min_voxel_2d.cwiseMax(Eigen::Vector2i::Zero());
                max_voxel_2d = max_voxel_2d.cwiseMin(
                    grid_size.head<2>().cast<int>() - Eigen::Vector2i::Ones());

                // 遍历2D包围盒
                for (int x = min_voxel_2d.x(); x <= max_voxel_2d.x(); ++x) {
                    for (int y = min_voxel_2d.y(); y <= max_voxel_2d.y(); ++y) {
                        Eigen::Vector3f point(
                            grid_origin.x() + (x + 0.5f) * voxel_size,
                            grid_origin.y() + (y + 0.5f) * voxel_size,
                            slice_z
                        );

                        float sdf = signed_distance_to_surface(point);
                        if (std::abs(sdf) <= config_.surface_threshold) {
                            grid.set(x, y, z, true);
                        }
                    }
                }
            }
        }
    }
    return true;
}

SurfaceVoxelizer::SurfaceVoxelizer(const VoxelizationConfig& config)
    : config_(config) {}

void SurfaceVoxelizer::set_surface(
    const std::vector<Eigen::Vector3f>& vertices,
    const std::vector<Eigen::Vector3f>& normals,
    const std::vector<Eigen::Vector3i>& faces) {
    vertices_ = vertices;
    normals_ = normals;
    faces_ = faces;
    compute_surface_properties();
}

void SurfaceVoxelizer::compute_surface_properties() {
    // 计算每个顶点的曲率
    vertex_curvatures_.resize(vertices_.size(), 0.0f);
    
    // 对每个顶点
    for (size_t i = 0; i < vertices_.size(); ++i) {
        std::vector<size_t> neighbor_indices;
        // 找到所有包含该顶点的面
        for (const auto& face : faces_) {
            if (face[0] == i || face[1] == i || face[2] == i) {
                if (face[0] != i) neighbor_indices.push_back(face[0]);
                if (face[1] != i) neighbor_indices.push_back(face[1]);
                if (face[2] != i) neighbor_indices.push_back(face[2]);
            }
        }

        // 计算平均曲率
        float curvature = 0.0f;
        for (size_t j : neighbor_indices) {
            float angle = std::acos(normals_[i].dot(normals_[j]));
            curvature += angle;
        }
        vertex_curvatures_[i] = curvature / neighbor_indices.size();
    }

    // 构建八叉树
    auto samples = generate_surface_samples();
    if (config_.use_adaptive_sampling) {
        adaptive_surface_sampling(samples);
    }
    octree_root_ = build_octree(samples);
}

float SurfaceVoxelizer::compute_local_feature_size(const Eigen::Vector3f &point) const
{
    return 0.0f;
}
bool SurfaceVoxelizer::is_sharp_feature(const Eigen::Vector3f &point, const Eigen::Vector3f &normal) const
{
    return false;
}
std::vector<SurfaceVoxelizer::SurfacePoint> SurfaceVoxelizer::generate_surface_samples() const
{
    std::vector<SurfacePoint> samples;
    
    // 对每个三角形面进行采样
    for (const auto& face : faces_) {
        const auto& v0 = vertices_[face[0]];
        const auto& v1 = vertices_[face[1]];
        const auto& v2 = vertices_[face[2]];
        
        const auto& n0 = normals_[face[0]];
        const auto& n1 = normals_[face[1]];
        const auto& n2 = normals_[face[2]];

        // 计算三角形面积以确定采样点数
        Eigen::Vector3f edge1 = v1 - v0;
        Eigen::Vector3f edge2 = v2 - v0;
        float area = edge1.cross(edge2).norm() * 0.5f;
        
        // 根据面积确定采样点数
        int num_samples = std::max(1, static_cast<int>(area / (config_.surface_threshold * config_.surface_threshold)));
        
        // 生成采样点
        for (int i = 0; i < num_samples; ++i) {
            float u = static_cast<float>(rand()) / RAND_MAX;
            float v = static_cast<float>(rand()) / RAND_MAX;
            if (u + v > 1.0f) {
                u = 1.0f - u;
                v = 1.0f - v;
            }
            float w = 1.0f - u - v;
            
            // 计算采样点位置和法向量
            SurfacePoint sample;
            sample.position = v0 * w + v1 * u + v2 * v;
            sample.normal = (n0 * w + n1 * u + n2 * v).normalized();
            
            // 插值计算曲率
            sample.curvature = vertex_curvatures_[face[0]] * w +
                             vertex_curvatures_[face[1]] * u +
                             vertex_curvatures_[face[2]] * v;
            
            samples.push_back(sample);
        }
    }
    
    return samples;
}

void SurfaceVoxelizer::adaptive_surface_sampling(std::vector<SurfacePoint>& samples) const {
    std::vector<SurfacePoint> additional_samples;
    
    // 在高曲率区域增加采样点
    for (const auto& sample : samples) {
        if (sample.curvature > config_.surface_threshold) {
            // 在高曲率点周围添加额外采样点
            for (int i = 0; i < 4; ++i) {
                SurfacePoint new_sample;
                float offset = config_.surface_threshold * 0.1f;
                new_sample.position = sample.position + 
                    Eigen::Vector3f::Random() * offset;
                new_sample.normal = sample.normal;
                new_sample.curvature = sample.curvature;
                additional_samples.push_back(new_sample);
            }
        }
    }
    
    samples.insert(samples.end(), additional_samples.begin(), additional_samples.end());
}
bool SurfaceVoxelizer::voxelize(VoxelGrid& grid) const {
    const float voxel_size = grid.resolution();
    const Eigen::Vector3f grid_origin = grid.origin();
    
    // 对每个体素进行处理
    #pragma omp parallel for collapse(3)
    for (int z = 0; z < grid.dimensions().z(); ++z) {
        for (int y = 0; y < grid.dimensions().y(); ++y) {
            for (int x = 0; x < grid.dimensions().x(); ++x) {
                // 计算体素中心点
                Eigen::Vector3f center = grid_origin + 
                    Eigen::Vector3f(x + 0.5f, y + 0.5f, z + 0.5f) * voxel_size;
                
                // 计算到表面的有符号距离
                float sdf = signed_distance_to_surface(center);
                
                // 根据距离场确定体素状态
                grid.set(x, y, z, sdf <= 0.0f);
            }
        }
    }
    
    return true;
}

float SurfaceVoxelizer::signed_distance_to_surface(const Eigen::Vector3f& point) const {
    std::vector<size_t> nearest_indices;
    query_nearest_points(point, octree_root_, nearest_indices);
    
    if (nearest_indices.empty()) {
        return std::numeric_limits<float>::max();
    }
    
    // 计算加权距离
    float min_dist = std::numeric_limits<float>::max();
    Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
    
    for (size_t idx : nearest_indices) {
        const auto& surface_point = vertices_[idx];
        const auto& normal = normals_[idx];
        
        float dist = (point - surface_point).dot(normal);
        min_dist = std::min(min_dist, std::abs(dist));
        avg_normal += normal;
    }
    
    avg_normal.normalize();
    float sign = (point - vertices_[nearest_indices[0]]).dot(avg_normal) > 0 ? 1.0f : -1.0f;
    
    return sign * min_dist;
}

std::shared_ptr<SurfaceVoxelizer::OctreeNode> SurfaceVoxelizer::build_octree(
    const std::vector<SurfacePoint>& points) const {
    // 实现八叉树构建逻辑
    auto root = std::make_shared<OctreeNode>();
    // ... 八叉树构建代码 ...
    return root;
}

void SurfaceVoxelizer::query_nearest_points(
    const Eigen::Vector3f& query,
    const std::shared_ptr<OctreeNode>& node,
    std::vector<size_t>& result) const {
    // 实现最近点查询逻辑
    // ... 最近点查询代码 ...
}

} // namespace VXZ