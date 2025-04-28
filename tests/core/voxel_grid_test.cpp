#include <gtest/gtest.h>
#include <core/voxel_grid.hpp>

using namespace VXZ;

class VoxelGridTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a test voxel grid with resolution 1.0 and bounds from (0,0,0) to (10,10,10)
        grid = std::make_unique<VoxelGrid>(1.0f, 
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Eigen::Vector3f(10.0f, 10.0f, 10.0f));
    }

    std::unique_ptr<VoxelGrid> grid;
};

TEST_F(VoxelGridTest, ConstructorTest) {
    EXPECT_FLOAT_EQ(grid->resolution(), 1.0f);
    EXPECT_EQ(grid->min_bounds(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(grid->max_bounds(), Eigen::Vector3f(10.0f, 10.0f, 10.0f));
    EXPECT_EQ(grid->dimensions(), Eigen::Vector3i(10, 10, 10));
}

TEST_F(VoxelGridTest, GridAccessTest) {
    // Test setting and getting voxels
    grid->set(5, 5, 5, true);
    EXPECT_TRUE(grid->get(5, 5, 5));
    
    grid->set(5, 5, 5, false);
    EXPECT_FALSE(grid->get(5, 5, 5));
}

TEST_F(VoxelGridTest, CoordinateConversionTest) {
    // Test world to grid conversion
    Eigen::Vector3f world_pos(5.5f, 5.5f, 5.5f);
    Eigen::Vector3i grid_pos = grid->world_to_grid(world_pos);
    EXPECT_EQ(grid_pos, Eigen::Vector3i(5, 5, 5));

    // Test grid to world conversion
    Eigen::Vector3f converted_world = grid->grid_to_world(grid_pos);
    EXPECT_NEAR(converted_world.x(), 5.5f, 0.001f);
    EXPECT_NEAR(converted_world.y(), 5.5f, 0.001f);
    EXPECT_NEAR(converted_world.z(), 5.5f, 0.001f);
}

TEST_F(VoxelGridTest, RegionOperationsTest) {
    // Test setting a region
    grid->set_region(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(2, 2, 2), true);
    
    // Verify the region was set
    for (int x = 0; x <= 2; x++) {
        for (int y = 0; y <= 2; y++) {
            for (int z = 0; z <= 2; z++) {
                EXPECT_TRUE(grid->get(x, y, z));
            }
        }
    }
}

TEST_F(VoxelGridTest, StatisticsTest) {
    // Fill some voxels
    grid->set(0, 0, 0, true);
    grid->set(1, 1, 1, true);
    grid->set(2, 2, 2, true);

    EXPECT_EQ(grid->count_occupied(), 3);
    EXPECT_FLOAT_EQ(grid->occupancy_rate(), 3.0f / (10 * 10 * 10));
}

TEST_F(VoxelGridTest, ValidationTest) {
    // Test valid positions
    EXPECT_TRUE(grid->is_valid_position(Eigen::Vector3i(0, 0, 0)));
    EXPECT_TRUE(grid->is_valid_position(Eigen::Vector3i(9, 9, 9)));

    // Test invalid positions
    EXPECT_FALSE(grid->is_valid_position(Eigen::Vector3i(-1, 0, 0)));
    EXPECT_FALSE(grid->is_valid_position(Eigen::Vector3i(10, 0, 0)));
    EXPECT_FALSE(grid->is_valid_position(Eigen::Vector3i(0, -1, 0)));
    EXPECT_FALSE(grid->is_valid_position(Eigen::Vector3i(0, 10, 0)));
    EXPECT_FALSE(grid->is_valid_position(Eigen::Vector3i(0, 0, -1)));
    EXPECT_FALSE(grid->is_valid_position(Eigen::Vector3i(0, 0, 10)));
} 