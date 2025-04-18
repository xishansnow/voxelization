#include <gtest/gtest.h>
#include <storage/storage.hpp>
#include <storage/svo.hpp>
#include <storage/svdag.hpp>
#include <storage/ssvdag.hpp>
#include <storage/openvdb.hpp>
#include <storage/nanovdb.hpp>
#include <core/voxel_grid.hpp>

using namespace VXZ;
using namespace voxelization;

class StorageTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple voxel grid for testing
        grid_ = VoxelGrid(8, 8, 8);
        for (int z = 0; z < 8; ++z) {
            for (int y = 0; y < 8; ++y) {
                for (int x = 0; x < 8; ++x) {
                    // Create a simple sphere
                    float dx = x - 4.0f;
                    float dy = y - 4.0f;
                    float dz = z - 4.0f;
                    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    grid_.set(x, y, z, dist <= 3.0f);
                }
            }
        }
    }

    VoxelGrid grid_;
};

TEST_F(StorageTest, SVOStorage) {
    SVOStorage storage(grid_);
    
    // Test save/load
    ASSERT_TRUE(storage.save("test.svo"));
    
    SVOStorage loaded;
    ASSERT_TRUE(loaded.load("test.svo"));
    
    // Test conversion
    VoxelGrid converted;
    ASSERT_TRUE(loaded.to_voxel_grid(converted));
    
    // Compare grids
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                EXPECT_EQ(grid_.get(x, y, z), converted.get(x, y, z));
            }
        }
    }
}

TEST_F(StorageTest, SVDAGStorage) {
    SVDAGStorage storage(grid_);
    
    // Test save/load
    ASSERT_TRUE(storage.save("test.svdag"));
    
    SVDAGStorage loaded;
    ASSERT_TRUE(loaded.load("test.svdag"));
    
    // Test conversion
    VoxelGrid converted;
    ASSERT_TRUE(loaded.to_voxel_grid(converted));
    
    // Compare grids
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                EXPECT_EQ(grid_.get(x, y, z), converted.get(x, y, z));
            }
        }
    }
}

TEST_F(StorageTest, SSVDAGStorage) {
    SSVDAGStorage storage(grid_);
    
    // Test save/load
    ASSERT_TRUE(storage.save("test.ssvdag"));
    
    SSVDAGStorage loaded;
    ASSERT_TRUE(loaded.load("test.ssvdag"));
    
    // Test conversion
    VoxelGrid converted;
    ASSERT_TRUE(loaded.to_voxel_grid(converted));
    
    // Compare grids
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                EXPECT_EQ(grid_.get(x, y, z), converted.get(x, y, z));
            }
        }
    }
}

TEST_F(StorageTest, OpenVDBStorage) {
    OpenVDBStorage storage(grid_);
    
    // Test save/load
    ASSERT_TRUE(storage.save("test.vdb"));
    
    OpenVDBStorage loaded;
    ASSERT_TRUE(loaded.load("test.vdb"));
    
    // Test conversion
    VoxelGrid converted;
    ASSERT_TRUE(loaded.to_voxel_grid(converted));
    
    // Compare grids
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                EXPECT_EQ(grid_.get(x, y, z), converted.get(x, y, z));
            }
        }
    }
}

TEST_F(StorageTest, NanoVDBStorage) {
    NanoVDBStorage storage(grid_);
    
    // Test save/load
    ASSERT_TRUE(storage.save("test.nvdb"));
    
    NanoVDBStorage loaded;
    ASSERT_TRUE(loaded.load("test.nvdb"));
    
    // Test conversion
    VoxelGrid converted;
    ASSERT_TRUE(loaded.to_voxel_grid(converted));
    
    // Compare grids
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                EXPECT_EQ(grid_.get(x, y, z), converted.get(x, y, z));
            }
        }
    }
}

TEST_F(StorageTest, StorageFactory) {
    // Test SVO
    auto svo = VoxelStorageFactory::create("svo", grid_);
    ASSERT_NE(svo, nullptr);
    
    // Test SVDAG
    auto svdag = VoxelStorageFactory::create("svdag", grid_);
    ASSERT_NE(svdag, nullptr);
    
    // Test SSVDAG
    auto ssvdag = VoxelStorageFactory::create("ssvdag", grid_);
    ASSERT_NE(ssvdag, nullptr);
    
    // Test OpenVDB
    auto openvdb = VoxelStorageFactory::create("openvdb", grid_);
    ASSERT_NE(openvdb, nullptr);
    
    // Test NanoVDB
    auto nanovdb = VoxelStorageFactory::create("nanovdb", grid_);
    ASSERT_NE(nanovdb, nullptr);
    
    // Test invalid type
    EXPECT_THROW(
        VoxelStorageFactory::create("invalid", grid_),
        std::invalid_argument
    );
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 