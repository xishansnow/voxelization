#include <gtest/gtest.h>
#include <core/voxel_grid.hpp>
#include <operator/UnionOperator.hpp>
#include <operator/IntersectionOperator.hpp>
#include <operator/DifferenceOperator.hpp>

using namespace VXZ;

class OperatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create two test voxel grids
        grid1 = std::make_unique<VoxelGrid>(1.0f, 
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Eigen::Vector3f(10.0f, 10.0f, 10.0f));
            
        grid2 = std::make_unique<VoxelGrid>(1.0f, 
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Eigen::Vector3f(10.0f, 10.0f, 10.0f));
            
        // Initialize with some test data
        // Grid1: A cube from (0,0,0) to (5,5,5)
        grid1->set_region(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(5, 5, 5), true);
        
        // Grid2: A cube from (3,3,3) to (8,8,8)
        grid2->set_region(Eigen::Vector3i(3, 3, 3), Eigen::Vector3i(8, 8, 8), true);
    }

    std::unique_ptr<VoxelGrid> grid1;
    std::unique_ptr<VoxelGrid> grid2;
};

TEST_F(OperatorTest, UnionOperatorTest) {
    UnionOperator union_op;
    auto result = union_op.apply(*grid1, *grid2);
    
    // Check that the union contains all voxels from both grids
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                bool expected = grid1->get(x, y, z) || grid2->get(x, y, z);
                EXPECT_EQ(result->get(x, y, z), expected);
            }
        }
    }
}

TEST_F(OperatorTest, IntersectionOperatorTest) {
    IntersectionOperator intersection_op;
    auto result = intersection_op.apply(*grid1, *grid2);
    
    // Check that the intersection contains only voxels present in both grids
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                bool expected = grid1->get(x, y, z) && grid2->get(x, y, z);
                EXPECT_EQ(result->get(x, y, z), expected);
            }
        }
    }
}

TEST_F(OperatorTest, DifferenceOperatorTest) {
    DifferenceOperator difference_op;
    auto result = difference_op.apply(*grid1, *grid2);
    
    // Check that the difference contains voxels from grid1 that are not in grid2
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                bool expected = grid1->get(x, y, z) && !grid2->get(x, y, z);
                EXPECT_EQ(result->get(x, y, z), expected);
            }
        }
    }
} 