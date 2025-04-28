#include <gtest/gtest.h>
#include <core/voxel_grid.hpp>
#include <storage/voxelstorage.hpp>

using namespace VXZ;

class StorageTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a test voxel grid
        grid = std::make_unique<VoxelGrid>(1.0f, 
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Eigen::Vector3f(10.0f, 10.0f, 10.0f));
            
        // Create a simple test shape (a cube)
        grid->set_region(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(5, 5, 5), true);
        
        // Initialize storage
        storage = std::make_unique<VoxelStorage>();
    }

    std::unique_ptr<VoxelGrid> grid;
    std::unique_ptr<VoxelStorage> storage;
};

TEST_F(StorageTest, SaveLoadTest) {
    // Test saving the voxel grid
    const std::string test_file = "test_voxel_grid.vxg";
    EXPECT_TRUE(storage->save(*grid, test_file));
    
    // Test loading the voxel grid
    auto loaded_grid = storage->load(test_file);
    EXPECT_NE(loaded_grid, nullptr);
    
    // Verify the loaded grid matches the original
    EXPECT_EQ(loaded_grid->resolution(), grid->resolution());
    EXPECT_EQ(loaded_grid->min_bounds(), grid->min_bounds());
    EXPECT_EQ(loaded_grid->max_bounds(), grid->max_bounds());
    EXPECT_EQ(loaded_grid->dimensions(), grid->dimensions());
    
    // Verify the voxel data matches
    for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
            for (int z = 0; z < 10; z++) {
                EXPECT_EQ(loaded_grid->get(x, y, z), grid->get(x, y, z));
            }
        }
    }
}

TEST_F(StorageTest, InvalidFileTest) {
    // Test loading from non-existent file
    auto loaded_grid = storage->load("non_existent_file.vxg");
    EXPECT_EQ(loaded_grid, nullptr);
}

TEST_F(StorageTest, FileFormatTest) {
    // Test saving with different file formats
    const std::string test_file_vxg = "test_voxel_grid.vxg";
    const std::string test_file_bin = "test_voxel_grid.bin";
    
    EXPECT_TRUE(storage->save(*grid, test_file_vxg));
    EXPECT_TRUE(storage->save(*grid, test_file_bin));
    
    // Test loading from different file formats
    auto loaded_grid_vxg = storage->load(test_file_vxg);
    auto loaded_grid_bin = storage->load(test_file_bin);
    
    EXPECT_NE(loaded_grid_vxg, nullptr);
    EXPECT_NE(loaded_grid_bin, nullptr);
} 