#include <gtest/gtest.h>
#include <core/voxel_grid.hpp>
#include <renderer/voxel_renderer.hpp>

using namespace VXZ;

class RendererTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a test voxel grid
        grid = std::make_unique<VoxelGrid>(1.0f, 
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Eigen::Vector3f(10.0f, 10.0f, 10.0f));
            
        // Create a simple test shape (a cube)
        grid->set_region(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(5, 5, 5), true);
        
        // Initialize renderer
        renderer = std::make_unique<VoxelRenderer>();
    }

    std::unique_ptr<VoxelGrid> grid;
    std::unique_ptr<VoxelRenderer> renderer;
};

TEST_F(RendererTest, BasicRenderingTest) {
    // Test that the renderer can initialize
    EXPECT_TRUE(renderer->initialize());
    
    // Test that the renderer can set the voxel grid
    EXPECT_TRUE(renderer->setVoxelGrid(*grid));
    
    // Test that the renderer can render
    EXPECT_TRUE(renderer->render());
}

TEST_F(RendererTest, CameraSettingsTest) {
    // Test camera position setting
    Eigen::Vector3f camera_pos(10.0f, 10.0f, 10.0f);
    EXPECT_TRUE(renderer->setCameraPosition(camera_pos));
    
    // Test camera target setting
    Eigen::Vector3f target_pos(5.0f, 5.0f, 5.0f);
    EXPECT_TRUE(renderer->setCameraTarget(target_pos));
    
    // Test camera up vector setting
    Eigen::Vector3f up_vector(0.0f, 1.0f, 0.0f);
    EXPECT_TRUE(renderer->setCameraUp(up_vector));
}

TEST_F(RendererTest, RenderingSettingsTest) {
    // Test setting rendering parameters
    EXPECT_TRUE(renderer->setResolution(800, 600));
    EXPECT_TRUE(renderer->setBackgroundColor(0.0f, 0.0f, 0.0f));
    EXPECT_TRUE(renderer->setVoxelColor(1.0f, 0.0f, 0.0f));
}

TEST_F(RendererTest, CleanupTest) {
    // Test that the renderer can clean up properly
    EXPECT_TRUE(renderer->cleanup());
} 