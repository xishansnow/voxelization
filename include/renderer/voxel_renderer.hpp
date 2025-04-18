#pragma once

#include <memory>
#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
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
    
    /**
     * @brief Get the GLFW window
     * @return Pointer to the GLFW window
     */
    GLFWwindow* get_window() const {
        return window_;
    }
    
    bool initialize();

    /**
     * @brief Render a voxel grid

     */
    void render(const VXZ::VoxelGrid& grid);

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




    // Camera parameters
    glm::vec3 cameraPos_ = glm::vec3(0.0f, 0.0f, 3.0f);
    glm::vec3 cameraFront_ = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 cameraUp_ = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 cameraRight_ = glm::vec3(1.0f, 0.0f, 0.0f);
    float cameraSpeed_ = 5.0f;
    bool rotateCamera_ = true;    

    // Mouse control
    float lastX_ = 400, lastY_ = 300;
    float yaw_ = -90.0f, pitch_ = 0.0f;
    bool firstMouse_ = true;

    // 计时器时间
    float deltaTime_ = 0.0f;
    float lastFrame_ = 0.0f;  
    

};

} // namespace VXZ 