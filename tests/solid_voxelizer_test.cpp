#include <gtest/gtest.h>
#include <voxelizer/SchwarzSolidVoxelizer.hpp>
#include <voxelizer/EisemannSolidVoxelizer.hpp>
#include <core/voxel_grid.hpp>
#include <vector>

using namespace VXZ;

static std::vector<Eigen::Vector3f> cube_vertices = {
    {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
    {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}
};
static std::vector<Eigen::Vector3i> cube_faces = {
    {{0,1,2}}, {{0,2,3}}, // bottom
    {{4,5,6}}, {{4,6,7}}, // top
    {{0,1,5}}, {{0,5,4}}, // front
    {{2,3,7}}, {{2,7,6}}, // back
    {{1,2,6}}, {{1,6,5}}, // right
    {{0,3,7}}, {{0,7,4}}  // left
};

TEST(SchwarzSolidVoxelizerTest, Cube) {
    VoxelGrid grid(0.5f, Eigen::Vector3f(0,0,0), Eigen::Vector3f(3,3,3));
    SchwarzSolidVoxelizer voxelizer;
    voxelizer.set_mesh(cube_vertices, cube_faces);
    ASSERT_TRUE(voxelizer.voxelize(grid));
    // 这里只检查接口可用
    EXPECT_EQ(grid.get_size_x(), 3);
}

TEST(EisemannSolidVoxelizerTest, Cube) {
    VoxelGrid grid(0.5f, Eigen::Vector3f(0,0,0), Eigen::Vector3f(3,3,3));
    EisemannSolidVoxelizer voxelizer;
    voxelizer.set_mesh(cube_vertices, cube_faces);
    ASSERT_TRUE(voxelizer.voxelize(grid));
    // 这里只检查接口可用
    EXPECT_EQ(grid.get_size_x(), 3);
}
