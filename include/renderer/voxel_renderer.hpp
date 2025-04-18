#pragma once

#include <memory>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <eigen3/Eigen/Dense>
#include "../core/voxel_grid.hpp"

namespace VXZ {

/**
 * @brief Class for rendering voxel grids
 */
class VoxelRenderer {
public:
    /**
     * @brief Constructor
     * @param width Window width
     * @param height Window height
     * @param title Window title
     */
    VoxelRenderer(int width, int height, const char* title);

    /**
     * @brief Destructor
     */
    ~VoxelRenderer();

    /**
     * @brief Initialize the renderer
     * @return true if successful, false otherwise
     */
    bool initialize();

    /**
     * @brief Render a voxel grid
     * @param grid The voxel grid to render
     * @param camera_pos Camera position
     * @param camera_target Camera target
     * @param camera_up Camera up vector
     */
    void render(const VXZ::VoxelGrid& grid,
                const Eigen::Vector3f& camera_pos,
                const Eigen::Vector3f& camera_target,
                const Eigen::Vector3f& camera_up);

    /**
     * @brief Check if the window should close
     * @return true if window should close, false otherwise
     */
    bool should_close() const;

    /**
     * @brief Process input events
     */
    void process_input();

private:
    /**
     * @brief Compile a shader
     * @param type Shader type
     * @param source Shader source code
     * @return Shader ID
     */
    GLuint compile_shader(GLenum type, const char* source);

    /**
     * @brief Create a shader program
     * @param vertex_source Vertex shader source
     * @param fragment_source Fragment shader source
     * @return Program ID
     */
    GLuint create_program(const char* vertex_source, const char* fragment_source);

    /**
     * @brief Create a cube mesh
     */
    void create_cube_mesh();

    GLFWwindow* window_;
    int width_;
    int height_;
    GLuint program_;
    GLuint vao_;
    GLuint vbo_;
    GLuint ebo_;
    std::vector<float> vertices_;
    std::vector<unsigned int> indices_;
};

} // namespace VXZ 