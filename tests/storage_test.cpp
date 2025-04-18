#include <gtest/gtest.h>
#include <storage/voxelstorage.hpp>
#include <storage/svo.hpp>
#include <core/voxel_grid.hpp>

using namespace VXZ;
using namespace VXZ;

class StorageTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple voxel grid for testing
        grid_ = VoxelGrid(1.0f, Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(8.0f, 8.0f, 8.0f));
        for (int z = 0; z < 8; ++z) {
            for (int y = 0; y < 8; ++y) {
                for (int x = 0; x < 8; ++x) {
                    // Create a simple sphere
                    float dx = x - 4.0f;
                    float dy = y - 4.0f;
                    float dz = z - 4.0f;
                    float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                    grid_.set(x, y, z, dist <= 3.0f);
                }
            }
        }
    }

    VoxelGrid grid_;
};

// 工厂方法：用于从 VoxelGrid 创建 SVOStorage 对象
std::unique_ptr<SVOStorage> createSVOStorageFromGrid(const VoxelGrid &grid) {
    auto storage = std::make_unique<SVOStorage>();
    if (!storage->from_voxel_grid(grid)) {
        throw std::runtime_error("Failed to initialize SVOStorage from VoxelGrid");
    }
    return storage;
}

TEST_F(StorageTest, SVOStorage) {
    // 使用工厂方法创建 SVOStorage 对象
    auto storage = createSVOStorageFromGrid(grid_);

    // Test save/load
    ASSERT_TRUE(storage->save("test.svo"));

    SVOStorage loaded;
    ASSERT_TRUE(loaded.load("test.svo"));

    // Test conversion
    // 提取方法以封装加载、转换和比较逻辑
    void validateLoadedVoxelGrid(const VoxelGrid &originalGrid, const SVOStorage &loadedStorage) {
        constexpr int GRID_DIMENSION = 8; // 引入常量以避免硬编码

        VoxelGrid loadedVoxelGrid; // 重命名为更具描述性的名称
        ASSERT_TRUE(loadedStorage.to_voxel_grid(loadedVoxelGrid));

        // 比较原始网格和加载后的网格
        for (int z = 0; z < GRID_DIMENSION; ++z) {
            for (int y = 0; y < GRID_DIMENSION; ++y) {
                for (int x = 0; x < GRID_DIMENSION; ++x) {
                    EXPECT_EQ(originalGrid.get(x, y, z), loadedVoxelGrid.get(x, y, z));
                }
            }
        }
    }

    // 测试代码调用提取的方法
    {
        // 使用工厂方法创建 SVOStorage 对象
        auto storage = createSVOStorageFromGrid(grid_);

        // 测试保存/加载
        ASSERT_TRUE(storage->save("test.svo"));

        SVOStorage loaded;
        ASSERT_TRUE(loaded.load("test.svo"));

        // 调用提取的方法进行验证
        validateLoadedVoxelGrid(grid_, loaded);
    }
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


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}