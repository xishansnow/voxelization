#include "renderer/voxel_renderer.hpp"
#include <iostream>
#include <fstream>
#include <sstream>

namespace VXZ {

// Vertex shader source
const char* vertex_shader_source = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 ourColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    ourColor = aColor;
}
)";

// Fragment shader source
const char* fragment_shader_source = R"(
#version 330 core
in vec3 ourColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(ourColor, 1.0);
}
)";

VoxelRenderer::VoxelRenderer(int width, int height, const char* title)
    : window_(nullptr), width_(width), height_(height), program_(0), vao_(0), vbo_(0), ebo_(0) {
        
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window_ = glfwCreateWindow(width_, height_, title, nullptr, nullptr);
    
    if (!window_) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window_);

    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        glfwTerminate();
        return;
    }
}

VoxelRenderer::~VoxelRenderer() {
    if (program_) glDeleteProgram(program_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (ebo_) glDeleteBuffers(1, &ebo_);
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
}

bool VoxelRenderer::initialize() {
    
    program_ = create_program(vertex_shader_source, fragment_shader_source);
    if (!program_) return false;

    create_cube_mesh();

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);

    glBindVertexArray(vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(float), vertices_.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int), indices_.data(), GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return true;
}

void VoxelRenderer::render(const VXZ::VoxelGrid& grid,
                          const Eigen::Vector3f& camera_pos,
                          const Eigen::Vector3f& camera_target,
                          const Eigen::Vector3f& camera_up) {
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(program_);

    // Create view matrix
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    view.block<3, 1>(0, 3) = -camera_pos;
    view.block<3, 3>(0, 0) = Eigen::Quaternionf::FromTwoVectors(
        Eigen::Vector3f::UnitZ(), (camera_target - camera_pos).normalized()).toRotationMatrix();

    // Create projection matrix
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float fov = 45.0f;
    float aspect = static_cast<float>(width_) / height_;
    float near = 0.1f;
    float far = 100.0f;
    projection(0, 0) = 1.0f / (aspect * std::tan(fov / 2.0f));
    projection(1, 1) = 1.0f / std::tan(fov / 2.0f);
    projection(2, 2) = -(far + near) / (far - near);
    projection(2, 3) = -2.0f * far * near / (far - near);
    projection(3, 2) = -1.0f;
    projection(3, 3) = 0.0f;

    // Set uniforms
    glUniformMatrix4fv(glGetUniformLocation(program_, "view"), 1, GL_FALSE, view.data());
    glUniformMatrix4fv(glGetUniformLocation(program_, "projection"), 1, GL_FALSE, projection.data());

    glBindVertexArray(vao_);

    // Render each active voxel
    for (int z = 0; z < grid.get_size_z(); ++z) {
        for (int y = 0; y < grid.get_size_y(); ++y) {
            for (int x = 0; x < grid.get_size_x(); ++x) {
                if (grid.get_voxel(x, y, z)) {
                    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
                    model.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
                    glUniformMatrix4fv(glGetUniformLocation(program_, "model"), 1, GL_FALSE, model.data());
                    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
                }
            }
        }
    }

    glBindVertexArray(0);
    glUseProgram(0);

    glfwSwapBuffers(window_);
}

bool VoxelRenderer::should_close() const {
    return glfwWindowShouldClose(window_);
}

void VoxelRenderer::process_input() {
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, true);
    }
}

GLuint VoxelRenderer::compile_shader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLchar infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Shader compilation error: " << infoLog << std::endl;
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

GLuint VoxelRenderer::create_program(const char* vertex_source, const char* fragment_source) {
    GLuint vertex_shader = compile_shader(GL_VERTEX_SHADER, vertex_source);
    GLuint fragment_shader = compile_shader(GL_FRAGMENT_SHADER, fragment_source);

    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        GLchar infoLog[512];
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        std::cerr << "Program linking error: " << infoLog << std::endl;
        glDeleteProgram(program);
        return 0;
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    return program;
}

void VoxelRenderer::create_cube_mesh() {
    // Cube vertices with position and color
    vertices_ = {
        // Front face
        -0.5f, -0.5f,  0.5f,  1.0f, 0.0f, 0.0f,  // bottom-left
         0.5f, -0.5f,  0.5f,  0.0f, 1.0f, 0.0f,  // bottom-right
         0.5f,  0.5f,  0.5f,  0.0f, 0.0f, 1.0f,  // top-right
        -0.5f,  0.5f,  0.5f,  1.0f, 1.0f, 0.0f,  // top-left
        // Back face
        -0.5f, -0.5f, -0.5f,  1.0f, 0.0f, 1.0f,  // bottom-left
         0.5f, -0.5f, -0.5f,  0.0f, 1.0f, 1.0f,  // bottom-right
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f, 1.0f,  // top-right
        -0.5f,  0.5f, -0.5f,  0.5f, 0.5f, 0.5f   // top-left
    };

    indices_ = {
        // Front face
        0, 1, 2, 2, 3, 0,
        // Back face
        4, 5, 6, 6, 7, 4,
        // Top face
        3, 2, 6, 6, 7, 3,
        // Bottom face
        0, 1, 5, 5, 4, 0,
        // Right face
        1, 2, 6, 6, 5, 1,
        // Left face
        0, 3, 7, 7, 4, 0
    };
}

} // namespace VXZ 