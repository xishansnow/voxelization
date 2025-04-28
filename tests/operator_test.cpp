#include <gtest/gtest.h>
#include <operator/grid_operator.hpp>
#include <core/voxel_grid.hpp>

namespace VXZ {
    
class OperatorTest : public ::testing::Test {
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

TEST_F(OperatorTest, SmoothOperator) {
    SmoothOperator op(2, 0.5f);
    VoxelGrid original = grid_;
    
    ASSERT_TRUE(op.apply(grid_));
    
    // Check that the grid has changed
    bool has_changed = false;
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                if (grid_.get(x, y, z) != original.get(x, y, z)) {
                    has_changed = true;
                    break;
                }
            }
            if (has_changed) break;
        }
        if (has_changed) break;
    }
    EXPECT_TRUE(has_changed);
}

TEST_F(OperatorTest, DilateOperator) {
    DilateOperator op(1);
    VoxelGrid original = grid_;
    
    ASSERT_TRUE(op.apply(grid_));
    
    // Check that the grid has changed and all original active voxels are still active
    bool has_changed = false;
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                if (grid_.get(x, y, z) != original.get(x, y, z)) {
                    has_changed = true;
                }
                if (original.get(x, y, z)) {
                    EXPECT_TRUE(grid_.get(x, y, z));
                }
            }
        }
    }
    EXPECT_TRUE(has_changed);
}

TEST_F(OperatorTest, ErodeOperator) {
    ErodeOperator op(1);
    VoxelGrid original = grid_;
    
    ASSERT_TRUE(op.apply(grid_));
    
    // Check that the grid has changed and all eroded voxels were originally active
    bool has_changed = false;
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                if (grid_.get(x, y, z) != original.get(x, y, z)) {
                    has_changed = true;
                }
                if (grid_.get(x, y, z)) {
                    EXPECT_TRUE(original.get(x, y, z));
                }
            }
        }
    }
    EXPECT_TRUE(has_changed);
}

TEST_F(OperatorTest, OffsetOperator) {
    OffsetOperator op(1.0f);
    VoxelGrid original = grid_;
    
    ASSERT_TRUE(op.apply(grid_));
    
    // Check that the grid has changed
    bool has_changed = false;
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                if (grid_.get(x, y, z) != original.get(x, y, z)) {
                    has_changed = true;
                    break;
                }
            }
            if (has_changed) break;
        }
        if (has_changed) break;
    }
    EXPECT_TRUE(has_changed);
}

TEST_F(OperatorTest, UnionOperator) {
    // Create another grid
    VoxelGrid other(8, 8, 8);
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                // Create a cube
                other.set(x, y, z, x >= 2 && x <= 5 && y >= 2 && y <= 5 && z >= 2 && z <= 5);
            }
        }
    }
    
    UnionOperator op(other);
    VoxelGrid original = grid_;
    
    ASSERT_TRUE(op.apply(grid_));
    
    // Check that the union is correct
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                EXPECT_EQ(grid_.get(x, y, z), original.get(x, y, z) || other.get(x, y, z));
            }
        }
    }
}

TEST_F(OperatorTest, IntersectionOperator) {
    // Create another grid
    VoxelGrid other(8, 8, 8);
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                // Create a cube
                other.set(x, y, z, x >= 2 && x <= 5 && y >= 2 && y <= 5 && z >= 2 && z <= 5);
            }
        }
    }
    
    IntersectionOperator op(other);
    VoxelGrid original = grid_;
    
    ASSERT_TRUE(op.apply(grid_));
    
    // Check that the intersection is correct
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                EXPECT_EQ(grid_.get(x, y, z), original.get(x, y, z) && other.get(x, y, z));
            }
        }
    }
}

TEST_F(OperatorTest, DifferenceOperator) {
    // Create another grid
    VoxelGrid other(8, 8, 8);
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                // Create a cube
                other.set(x, y, z, x >= 2 && x <= 5 && y >= 2 && y <= 5 && z >= 2 && z <= 5);
            }
        }
    }
    
    DifferenceOperator op(other);
    VoxelGrid original = grid_;
    
    ASSERT_TRUE(op.apply(grid_));
    
    // Check that the difference is correct
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                EXPECT_EQ(grid_.get(x, y, z), original.get(x, y, z) && !other.get(x, y, z));
            }
        }
    }
}

TEST_F(OperatorTest, OpeningOperator) {
    OpeningOperator op(1);
    VoxelGrid original = grid_;
    
    ASSERT_TRUE(op.apply(grid_));
    
    // Check that the grid has changed
    bool has_changed = false;
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                if (grid_.get(x, y, z) != original.get(x, y, z)) {
                    has_changed = true;
                    break;
                }
            }
            if (has_changed) break;
        }
        if (has_changed) break;
    }
    EXPECT_TRUE(has_changed);
}

TEST_F(OperatorTest, ClosingOperator) {
    ClosingOperator op(1);
    VoxelGrid original = grid_;
    
    ASSERT_TRUE(op.apply(grid_));
    
    // Check that the grid has changed
    bool has_changed = false;
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                if (grid_.get(x, y, z) != original.get(x, y, z)) {
                    has_changed = true;
                    break;
                }
            }
            if (has_changed) break;
        }
        if (has_changed) break;
    }
    EXPECT_TRUE(has_changed);
}

TEST_F(OperatorTest, DistanceTransformOperator) {
    DistanceTransformOperator op(10.0f);
    VoxelGrid original = grid_;
    
    ASSERT_TRUE(op.apply(grid_));
    
    // Check that distances are computed correctly
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                if (original.get(x, y, z)) {
                    EXPECT_EQ(grid_.get(x, y, z), 0.0f);
                } else {
                    EXPECT_GE(grid_.get(x, y, z), 0.0f);
                    EXPECT_LE(grid_.get(x, y, z), 10.0f);
                }
            }
        }
    }
}

TEST_F(OperatorTest, ConnectedComponentsOperator) {
    // Create a grid with two disconnected components
    VoxelGrid test_grid(8, 8, 8);
    for (int z = 0; z < 4; ++z) {
        for (int y = 0; y < 4; ++y) {
            for (int x = 0; x < 4; ++x) {
                test_grid.set(x, y, z, true);
            }
        }
    }
    for (int z = 4; z < 8; ++z) {
        for (int y = 4; y < 8; ++y) {
            for (int x = 4; x < 8; ++x) {
                test_grid.set(x, y, z, true);
            }
        }
    }
    
    ConnectedComponentsOperator op(6);
    ASSERT_TRUE(op.apply(test_grid));
    
    // Check that components are labeled correctly
    int label1 = test_grid.get(0, 0, 0);
    int label2 = test_grid.get(7, 7, 7);
    EXPECT_NE(label1, label2);
    EXPECT_EQ(op.get_component_count(), 2);
    
    // Check that all voxels in each component have the same label
    for (int z = 0; z < 4; ++z) {
        for (int y = 0; y < 4; ++y) {
            for (int x = 0; x < 4; ++x) {
                EXPECT_EQ(test_grid.get(x, y, z), label1);
            }
        }
    }
    for (int z = 4; z < 8; ++z) {
        for (int y = 4; y < 8; ++y) {
            for (int x = 4; x < 8; ++x) {
                EXPECT_EQ(test_grid.get(x, y, z), label2);
            }
        }
    }
}

TEST_F(OperatorTest, FillOperator) {
    // Create a grid with a hollow sphere
    VoxelGrid test_grid(8, 8, 8);
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                float dx = x - 4.0f;
                float dy = y - 4.0f;
                float dz = z - 4.0f;
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                test_grid.set(x, y, z, dist >= 2.0f && dist <= 3.0f);
            }
        }
    }
    
    // Fill from the center
    FillOperator op(Eigen::Vector3i(4, 4, 4), 6);
    ASSERT_TRUE(op.apply(test_grid));
    
    // Check that the interior is filled
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                float dx = x - 4.0f;
                float dy = y - 4.0f;
                float dz = z - 4.0f;
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < 2.0f) {
                    EXPECT_TRUE(test_grid.get(x, y, z));
                }
            }
        }
    }
}

TEST_F(OperatorTest, InterpolationOperator) {
    // Create a grid with a linear gradient
    VoxelGrid test_grid(8, 8, 8);
    for (int z = 0; z < 8; ++z) {
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                test_grid.set(x, y, z, static_cast<float>(x + y + z) / 21.0f);
            }
        }
    }

    // Test interpolation at various positions
    {
        // Test at exact voxel position
        InterpolationOperator op(Eigen::Vector3f(4.0f, 4.0f, 4.0f));
        ASSERT_TRUE(op.apply(test_grid));
        EXPECT_FLOAT_EQ(op.get_value(), test_grid.get(4, 4, 4));
    }

    {
        // Test at position between voxels
        InterpolationOperator op(Eigen::Vector3f(4.5f, 4.5f, 4.5f));
        ASSERT_TRUE(op.apply(test_grid));
        float expected = (test_grid.get(4, 4, 4) + test_grid.get(4, 4, 5) +
                         test_grid.get(4, 5, 4) + test_grid.get(4, 5, 5) +
                         test_grid.get(5, 4, 4) + test_grid.get(5, 4, 5) +
                         test_grid.get(5, 5, 4) + test_grid.get(5, 5, 5)) / 8.0f;
        EXPECT_FLOAT_EQ(op.get_value(), expected);
    }

    {
        // Test at edge of grid
        InterpolationOperator op(Eigen::Vector3f(7.5f, 7.5f, 7.5f));
        ASSERT_TRUE(op.apply(test_grid));
        float expected = (test_grid.get(7, 7, 7) + test_grid.get(7, 7, 7) +
                         test_grid.get(7, 7, 7) + test_grid.get(7, 7, 7) +
                         test_grid.get(7, 7, 7) + test_grid.get(7, 7, 7) +
                         test_grid.get(7, 7, 7) + test_grid.get(7, 7, 7)) / 8.0f;
        EXPECT_FLOAT_EQ(op.get_value(), expected);
    }

    {
        // Test out of bounds
        InterpolationOperator op(Eigen::Vector3f(-1.0f, -1.0f, -1.0f));
        EXPECT_FALSE(op.apply(test_grid));
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 
