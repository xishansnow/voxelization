#include <gtest/gtest.h>
#include <core/voxel_grid.hpp>
#include <voxelizer/box_voxelizer.hpp>
#include <voxelizer/sphere_voxelizer.hpp>
#include <voxelizer/cylinder_voxelizer.hpp>

using namespace VXZ;

class VoxelizerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a test voxel grid
        grid = std::make_unique<VoxelGrid>(1.0f, 
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Eigen::Vector3f(10.0f, 10.0f, 10.0f));
    }

    std::unique_ptr<VoxelGrid> grid;
};
TEST_F(VoxelizerTest, BoxVoxelizerTest) {
    VXZ::BoxVoxelizerCPU box_voxelizer(); // Pass resolution to constructor
    
    // Create a box from (2,2,2) to (7,7,7)
    Eigen::Vector3f min_point(2.0f, 2.0f, 2.0f);
    Eigen::Vector3f max_point(7.0f, 7.0f, 7.0f);
    
    auto result = box_voxelizer.voxelize(*grid, min_point, max_point);
    EXPECT_NE(result, nullptr);
    
    // Verify the box was voxelized correctly
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                bool expected = (x >= 2 && x <= 7 && y >= 2 && y <= 7 && z >= 2 && z <= 7);
                EXPECT_EQ(result->get(x, y, z), expected);
            }
        }
    }
}

TEST_F(VoxelizerTest, SphereVoxelizerTest) {
    VXZ::SphereVoxelizerCPU sphere_voxelizer;
    
    // Create a sphere at (5,5,5) with radius 3
    Eigen::Vector3f center(5.0f, 5.0f, 5.0f);
    float radius = 3.0f;
    
    auto result = sphere_voxelizer.voxelize(*grid, center, radius);
    EXPECT_NE(result, nullptr);
    
    // Verify the sphere was voxelized correctly
    // We'll check a few key points
    EXPECT_TRUE(result->get(5, 5, 5));  // Center
    EXPECT_TRUE(result->get(5, 5, 7));  // Top
    EXPECT_TRUE(result->get(5, 5, 3));  // Bottom
    EXPECT_TRUE(result->get(7, 5, 5));  // Right
    EXPECT_TRUE(result->get(3, 5, 5));  // Left
    EXPECT_TRUE(result->get(5, 7, 5));  // Front
    EXPECT_TRUE(result->get(5, 3, 5));  // Back
    
    // Points outside the sphere should be empty
    EXPECT_FALSE(result->get(0, 0, 0));
    EXPECT_FALSE(result->get(9, 9, 9));
}

TEST_F(VoxelizerTest, CylinderVoxelizerTest) {
    VXZ::CylinderVoxelizerCPU cylinder_voxelizer;
    
    // Create a cylinder from (2,2,2) to (2,2,7) with radius 2
    Eigen::Vector3f start_point(2.0f, 2.0f, 2.0f);
    Eigen::Vector3f end_point(2.0f, 2.0f, 7.0f);
    float radius = 2.0f;
    
    auto result = cylinder_voxelizer.voxelize(*grid, start_point, end_point, radius);
    EXPECT_NE(result, nullptr);
    
    // Verify the cylinder was voxelized correctly
    // We'll check a few key points
    EXPECT_TRUE(result->get(2, 2, 2));  // Start point
    EXPECT_TRUE(result->get(2, 2, 7));  // End point
    EXPECT_TRUE(result->get(2, 4, 4));  // Side point
    EXPECT_TRUE(result->get(4, 2, 4));  // Side point
    
    // Points outside the cylinder should be empty
    EXPECT_FALSE(result->get(0, 0, 0));
    EXPECT_FALSE(result->get(9, 9, 9));
} 