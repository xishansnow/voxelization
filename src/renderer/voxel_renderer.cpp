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
    layout (location = 2) in vec3 aNormal;
    
    out vec3 ourColor;
    out vec3 Normal;
    out vec3 FragPos;
    
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    
    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
        ourColor = aColor;
        Normal = mat3(transpose(inverse(model))) * aNormal;
        FragPos = vec3(model * vec4(aPos, 1.0));
    }
)";

// Fragment shader source
const char* fragment_shader_source = R"(
    #version 330 core
    in vec3 ourColor;
    in vec3 Normal;
    in vec3 FragPos;
    
    out vec4 FragColor;
    
    uniform vec3 lightPos;
    uniform vec3 viewPos;
    uniform vec3 lightColor;
    
    void main() {
        // Ambient
        float ambientStrength = 0.1;
        vec3 ambient = ambientStrength * lightColor;
        
        // Diffuse
        vec3 norm = normalize(Normal);
        vec3 lightDir = normalize(lightPos - FragPos);
        float diff = max(dot(norm, lightDir), 0.0);
        vec3 diffuse = diff * lightColor;
        
        // Specular
        float specularStrength = 0.5;
        vec3 viewDir = normalize(viewPos - FragPos);
        vec3 reflectDir = reflect(-lightDir, norm);
        float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
        vec3 specular = specularStrength * spec * lightColor;
        
        vec3 result = (ambient + diffuse + specular) * ourColor;
        FragColor = vec4(result, 1.0);
    }
)";

VoxelRenderer::VoxelRenderer(int width, int height, const char* title)
    : window_(nullptr), width_(width), height_(height), program_(0), vao_(0), vbo_(0), ebo_(0) {

    // 初始化 GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }
    // 设置 OpenGL 版本
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // 创建窗口
    window_ = glfwCreateWindow(width_, height_, title, nullptr, nullptr);
    
    if (!window_) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }

    // 设置当前上下文
    glfwMakeContextCurrent(window_);

    // 设置窗口大小回调
    glfwSetFramebufferSizeCallback(window_, [](GLFWwindow* window, int width, int height) {
        glViewport(0, 0, width, height);
    });

    // 设置鼠标回调
    glfwSetCursorPosCallback(window_, [](GLFWwindow* window, double xpos, double ypos) {
        VoxelRenderer* renderer = static_cast<VoxelRenderer*>(glfwGetWindowUserPointer(window));
        if (renderer->firstMouse_) {
            renderer->lastX_ = xpos;
            renderer->lastY_ = ypos;
            renderer->firstMouse_ = false;
        }
        
        float xoffset = xpos - renderer->lastX_;
        float yoffset = renderer->lastY_ - ypos; // reversed: y ranges bottom to top
        renderer->lastX_ = xpos;
        renderer->lastY_ = ypos;
        
        float sensitivity = 0.1f;
        xoffset *= sensitivity;
        yoffset *= sensitivity;
        
        renderer->yaw_ += xoffset;
        renderer->pitch_ += yoffset;
        
        // Constrain pitch
        if (renderer->pitch_ > 89.0f)
            renderer->pitch_ = 89.0f;
        if (renderer->pitch_ < -89.0f)
            renderer->pitch_ = -89.0f;
            
        // Update camera front vector
        glm::vec3 direction;
        direction.x = cos(glm::radians(renderer->yaw_)) * cos(glm::radians(renderer->pitch_));
        direction.y = sin(glm::radians(renderer->pitch_));
        direction.z = sin(glm::radians(renderer->yaw_)) * cos(glm::radians(renderer->pitch_));
        renderer->cameraFront_ = glm::normalize(direction);
    });
    
    glfwSetScrollCallback(window_, [](GLFWwindow* window, double xoffset, double yoffset) {
        VoxelRenderer* renderer = static_cast<VoxelRenderer*>(glfwGetWindowUserPointer(window));
        renderer->cameraPos_ += renderer->cameraFront_ * (float)yoffset * 0.5f;
    });
    
    // Set user pointer to this instance for callbacks
    glfwSetWindowUserPointer(window_, this);
    
    // Set input mode for mouse
    glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);


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

/**
 * @brief 初始化渲染器
 * 在渲染器初始化时，需要完成着色器程序的创建，立方体网格的创建立方体网格的创建，缓冲区的创建
 * @return 是否初始化成功
 */
bool VoxelRenderer::initialize() {
    
    // 创建着色器程序
    program_ = create_program(vertex_shader_source, fragment_shader_source);
    if (!program_) return false;

    // 创建立方体网格
    create_cube_mesh();

    // 创建缓冲区，
    glGenVertexArrays(1, &vao_); // 创建顶点数组对象 
    glGenBuffers(1, &vbo_); // 创建顶点缓冲区
    glGenBuffers(1, &ebo_); // 创建索引缓冲区

    // 绑定缓冲区对象
    glBindVertexArray(vao_); // 绑定顶点数组对象
    glBindBuffer(GL_ARRAY_BUFFER, vbo_); // 绑定顶点缓冲区
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(float), vertices_.data(), GL_STATIC_DRAW); // 设置顶点缓冲区数据
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_); // 绑定索引缓冲区
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int), indices_.data(), GL_STATIC_DRAW); // 设置索引缓冲区数据

    // 设置顶点属性指针
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // 设置颜色属性指针
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // 解绑缓冲区
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return true;
}

/**
 * @brief 渲染体素网格
 * 在渲染时，需要完成背景颜色的设置，颜色缓冲区和深度缓冲区的清除，着色器程序的设定，视图矩阵的创建，投影矩阵的创建，光照属性的设定，立方体的渲染
 * @param grid 体素网格
 */
void VoxelRenderer::render(const VXZ::VoxelGrid& grid)
{                            
    // 设置背景颜色
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    // 清除颜色缓冲区和深度缓冲区
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 设定着色器程序
    if (program_) glUseProgram(program_);    

    // 创建视图矩阵      
  
        // Create transformations
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::lookAt(cameraPos_, cameraPos_ + cameraFront_, cameraUp_);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);


        // Set uniforms
        glUniformMatrix4fv(glGetUniformLocation(program_, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(program_, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(program_, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

        // Set light properties
        glUniform3f(glGetUniformLocation(program_, "lightPos"), 2.0f, 2.0f, 2.0f);
        glUniform3f(glGetUniformLocation(program_, "viewPos"), cameraPos_.x, cameraPos_.y, cameraPos_.z);
        glUniform3f(glGetUniformLocation(program_, "lightColor"), 1.0f, 1.0f, 1.0f);

        // Render cube
        glBindVertexArray(vao_);
        glDrawArrays(GL_TRIANGLES, 0, 36);

    // 渲染每个激活的体素
    // for (int z = 0; z < grid.get_size_z(); ++z) {
    //     for (int y = 0; y < grid.get_size_y(); ++y) {
    //         for (int x = 0; x < grid.get_size_x(); ++x) {
    //             if (grid.get_voxel(x, y, z)) {
    //                 Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    //                 model.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
    //                 glUniformMatrix4fv(glGetUniformLocation(program_, "model"), 1, GL_FALSE, model.data());
    //                 glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
    //             }
    //         }
    //     }
    // }

    // // 解绑顶点数组对象
    // glBindVertexArray(0);
    // // 解绑着色器程序
    // glUseProgram(0);

    // 交换缓冲区
    glfwPollEvents();
    glfwSwapBuffers(window_);
 
}

/**
 * @brief 检查是否关闭窗口
 * 
 * @return 是否关闭窗口
 */
bool VoxelRenderer::should_close() const {
    return glfwWindowShouldClose(window_);
}

void VoxelRenderer::process_input() {
    float currentFrame = glfwGetTime();
    deltaTime_ = currentFrame - lastFrame_;
    lastFrame_ = currentFrame;

    // 检查是否关闭窗口
    if (glfwGetKey(window_, GLFW_KEY_Q) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, true);
    }

    // 相机控制 
    cameraSpeed_ = 2.5f * deltaTime_;

    if (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos_ += cameraSpeed_ * cameraFront_;
    if (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos_ -= cameraSpeed_ * cameraFront_;
    if (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos_ -= glm::normalize(glm::cross(cameraFront_, cameraUp_)) * cameraSpeed_;
    if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos_ += glm::normalize(glm::cross(cameraFront_, cameraUp_)) * cameraSpeed_;
    if (glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS)
        rotateCamera_ = !rotateCamera_;
    if (rotateCamera_) {
        glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    } else {
        glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
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

// 创建着色器程序
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