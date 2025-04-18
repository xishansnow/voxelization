#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <vector>
#include <memory>
#include "core/voxel_grid.hpp"

namespace VXZ {

class VolumeRenderer {
public:
    VolumeRenderer(int width, int height, const std::string& title);
    ~VolumeRenderer();

    bool initialize();
    bool should_close() const;
    void process_input();
    void render(const VoxelGrid& grid);

private:
    // Window and OpenGL context
    GLFWwindow* window_;
    int width_;
    int height_;

    // Shader program
    GLuint shader_program_;
    GLuint vertex_shader_;
    GLuint fragment_shader_;

    // VAO and VBO for fullscreen quad
    GLuint vao_;
    GLuint vbo_;

    // 3D texture for volume data
    GLuint volume_texture_;

    // Transfer function texture
    GLuint transfer_function_texture_;

    // Camera parameters
    glm::vec3 camera_pos_;
    glm::vec3 camera_front_;
    glm::vec3 camera_up_;
    float fov_;
    float aspect_ratio_;
    float near_plane_;
    float far_plane_;

    // Shader source code
    static const char* vertex_shader_source_;
    static const char* fragment_shader_source_;

    // Helper functions
    bool compile_shader(GLuint shader, const char* source);
    bool link_program(GLuint program);
    void create_fullscreen_quad();
    void create_volume_texture(const VoxelGrid& grid);
    void create_transfer_function();
    void update_camera();
};

} // namespace VXZ 