#include <gtest/gtest.h>
#include <voxelizer/LevelSetVoxelizer.hpp>
#include <voxelizer/SDFVoxelizer.hpp>
#include <voxelizer/ImplicitSurfaceVoxelizer.hpp>
#include <core/voxel_grid.hpp>

using namespace VXZ;

TEST(LevelSetVoxelizerTest, Sphere) {
    VoxelGrid grid(1.0f, Eigen::Vector3f(0,0,0), Eigen::Vector3f(8,8,8));
    LevelSetVoxelizer voxelizer;
    ASSERT_TRUE(voxelizer.voxelize(grid));
    // 检查中心点是否为true
    EXPECT_TRUE(grid.get(4,4,4));
}

TEST(SDFVoxelizerTest, Sphere) {
    VoxelGrid grid(1.0f, Eigen::Vector3f(0,0,0), Eigen::Vector3f(8,8,8));
    SDFVoxelizer voxelizer;
    ASSERT_TRUE(voxelizer.voxelize(grid));
    // 检查中心点是否为true
    EXPECT_TRUE(grid.get(4,4,4));
}

TEST(ImplicitSurfaceVoxelizerTest, Sphere) {
    VoxelGrid grid(1.0f, Eigen::Vector3f(0,0,0), Eigen::Vector3f(8,8,8));
    ImplicitSurfaceVoxelizer voxelizer;
    ASSERT_TRUE(voxelizer.voxelize(grid));
    // 检查中心点是否为true
    EXPECT_TRUE(grid.get(4,4,4));
}

