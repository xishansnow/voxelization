#include "renderer/volume_renderer.hpp"
#include <iostream>
#include <vector>

namespace VXZ {

/**
 * @brief 顶点着色器
 * 
 * 顶点着色器是用于将顶点坐标转换为裁剪空间坐标的着色器
*/

const char* VolumeRenderer::vertex_shader_source_ = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    layout (location = 1) in vec2 aTexCoord;

    out vec2 TexCoord;

    void main() {
        gl_Position = vec4(aPos, 0.0, 1.0);
        TexCoord = aTexCoord;
    }
)";

/**
 * @brief 片段着色器
 * 
 * 片段着色器是用于计算每个像素的颜色值的着色器
*/
const char* VolumeRenderer::fragment_shader_source_ = R"(
    #version 330 core
    in vec2 TexCoord;
    out vec4 FragColor;

    uniform sampler3D volumeTexture;
    uniform sampler1D transferFunction;
    uniform mat4 invViewMatrix;
    uniform mat4 invProjectionMatrix;
    uniform vec3 cameraPos;
    uniform float stepSize;

    void main() {
        // Calculate ray direction in world space
        vec4 ray_clip = vec4(TexCoord * 2.0 - 1.0, -1.0, 1.0);
        vec4 ray_eye = invProjectionMatrix * ray_clip;
        ray_eye = vec4(ray_eye.xy, -1.0, 0.0);
        vec3 ray_world = (invViewMatrix * ray_eye).xyz;
        ray_world = normalize(ray_world);

        // Ray casting parameters
        vec3 ray_pos = cameraPos;
        float ray_length = 0.0;
        vec4 color = vec4(0.0);

        // Ray casting loop
        for (int i = 0; i < 1000; i++) {
            // Check if ray is still in volume
            if (ray_pos.x < 0.0 || ray_pos.x > 1.0 ||
                ray_pos.y < 0.0 || ray_pos.y > 1.0 ||
                ray_pos.z < 0.0 || ray_pos.z > 1.0) {
                break;
            }

            // Sample volume
            float density = texture(volumeTexture, ray_pos).r;
            
            // Apply transfer function
            vec4 sample_color = texture(transferFunction, density);
            
            // Accumulate color
            color = color + sample_color * (1.0 - color.a);

            // Early ray termination
            if (color.a > 0.99) {
                break;
            }

            // Advance ray
            ray_pos += ray_world * stepSize;
            ray_length += stepSize;
        }

        FragColor = color;
    }
)";

VolumeRenderer::VolumeRenderer(int width, int height, const std::string& title)
    : width_(width), height_(height), window_(nullptr),
      shader_program_(0), vertex_shader_(0), fragment_shader_(0),
      vao_(0), vbo_(0), volume_texture_(0), transfer_function_texture_(0),
      camera_pos_(0.0f, 0.0f, 2.0f),
      camera_front_(0.0f, 0.0f, -1.0f),
      camera_up_(0.0f, 1.0f, 0.0f),
      fov_(45.0f), aspect_ratio_(float(width) / float(height)),
      near_plane_(0.1f), far_plane_(100.0f) {
}

VolumeRenderer::~VolumeRenderer() {
    if (window_) {
        glfwDestroyWindow(window_);
    }
    glfwTerminate();
}

/**
 * @brief 初始化渲染器
 * 在初始化渲染器时，需要完成GLFW的初始化，GLFW窗口的创建，着色器程序的创建，立方体网格的创建立方体网格的创建，缓冲区的创建
 * @return 是否初始化成功
 */
bool VolumeRenderer::initialize() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // Configure GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window
    window_ = glfwCreateWindow(width_, height_, "Volume Renderer", NULL, NULL);
    if (!window_) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return false;
    }

    // Create shaders
    vertex_shader_ = glCreateShader(GL_VERTEX_SHADER);
    fragment_shader_ = glCreateShader(GL_FRAGMENT_SHADER);
    shader_program_ = glCreateProgram();

    if (!compile_shader(vertex_shader_, vertex_shader_source_) ||
        !compile_shader(fragment_shader_, fragment_shader_source_) ||
        !link_program(shader_program_)) {
        return false;
    }

    // Create fullscreen quad
    create_fullscreen_quad();

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    return true;
}

bool VolumeRenderer::should_close() const {
    return glfwWindowShouldClose(window_);
}

void VolumeRenderer::process_input() {
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window_, true);

    float cameraSpeed = 0.05f;
    if (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS)
        camera_pos_ += cameraSpeed * camera_front_;
    if (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS)
        camera_pos_ -= cameraSpeed * camera_front_;
    if (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS)
        camera_pos_ -= glm::normalize(glm::cross(camera_front_, camera_up_)) * cameraSpeed;
    if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS)
        camera_pos_ += glm::normalize(glm::cross(camera_front_, camera_up_)) * cameraSpeed;
}

/**
 * 渲染函数
 * 
 * 在渲染时，需要完成背景颜色的设置，颜色缓冲区和深度缓冲区的清除，着色器程序的设定，视图矩阵的创建，投影矩阵的创建，光照属性的设定，立方体的渲染
 * 
 * 
*/
void VolumeRenderer::render(const VoxelGrid& grid) {
    // Clear screen
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 设置着色器 Use shader program
    glUseProgram(shader_program_);

    // 更新相机
    update_camera();

    // 设置 uniforms
    glm::mat4 view = glm::lookAt(camera_pos_, camera_pos_ + camera_front_, camera_up_);
    glm::mat4 projection = glm::perspective(glm::radians(fov_), aspect_ratio_, near_plane_, far_plane_);
    glm::mat4 invView = glm::inverse(view);
    glm::mat4 invProjection = glm::inverse(projection);

    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "invViewMatrix"), 1, GL_FALSE, glm::value_ptr(invView));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "invProjectionMatrix"), 1, GL_FALSE, glm::value_ptr(invProjection));
    glUniform3fv(glGetUniformLocation(shader_program_, "cameraPos"), 1, glm::value_ptr(camera_pos_));
    glUniform1f(glGetUniformLocation(shader_program_, "stepSize"), 0.001f);

    // 绑定纹理
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_3D, volume_texture_);
    glUniform1i(glGetUniformLocation(shader_program_, "volumeTexture"), 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_1D, transfer_function_texture_);
    glUniform1i(glGetUniformLocation(shader_program_, "transferFunction"), 1);

    // 渲染全屏四边形
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    // 交换缓冲区和轮询事件
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

bool VolumeRenderer::compile_shader(GLuint shader, const char* source) {
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLchar infoLog[512];
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cerr << "Shader compilation failed: " << infoLog << std::endl;
        return false;
    }
    return true;
}

bool VolumeRenderer::link_program(GLuint program) {
    glAttachShader(program, vertex_shader_);
    glAttachShader(program, fragment_shader_);
    glLinkProgram(program);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        GLchar infoLog[512];
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        std::cerr << "Program linking failed: " << infoLog << std::endl;
        return false;
    }
    return true;
}

void VolumeRenderer::create_fullscreen_quad() {
    float vertices[] = {
        -1.0f, -1.0f, 0.0f, 0.0f,
         1.0f, -1.0f, 1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f, 1.0f,
         1.0f,  1.0f, 1.0f, 1.0f
    };

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
}

void VolumeRenderer::create_volume_texture(const VoxelGrid& grid) {
    glGenTextures(1, &volume_texture_);
    glBindTexture(GL_TEXTURE_3D, volume_texture_);

    // Set texture parameters
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Get grid dimensions
    Eigen::Vector3i dims = grid.dimensions();
    
    // Create volume data
    std::vector<float> volume_data(dims.x() * dims.y() * dims.z());
    for (int z = 0; z < dims.z(); z++) {
        for (int y = 0; y < dims.y(); y++) {
            for (int x = 0; x < dims.x(); x++) {
                volume_data[z * dims.x() * dims.y() + y * dims.x() + x] = 
                    grid.get(Eigen::Vector3i(x, y, z)) ? 1.0f : 0.0f;
            }
        }
    }

    // Upload texture data
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, dims.x(), dims.y(), dims.z(), 0, GL_RED, GL_FLOAT, volume_data.data());
}

void VolumeRenderer::create_transfer_function() {
    glGenTextures(1, &transfer_function_texture_);
    glBindTexture(GL_TEXTURE_1D, transfer_function_texture_);

    // Set texture parameters
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Create transfer function data (RGBA)
    std::vector<float> tf_data(256 * 4);
    for (int i = 0; i < 256; i++) {
        float t = float(i) / 255.0f;
        // Simple transfer function: white to red
        tf_data[i * 4 + 0] = 1.0f;     // R
        tf_data[i * 4 + 1] = 1.0f - t; // G
        tf_data[i * 4 + 2] = 1.0f - t; // B
        tf_data[i * 4 + 3] = t;        // A
    }

    // Upload texture data
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, 256, 0, GL_RGBA, GL_FLOAT, tf_data.data());
}

void VolumeRenderer::update_camera() {
    // Update camera based on mouse input
    static double lastX = width_ / 2.0;
    static double lastY = height_ / 2.0;
    static float yaw = -90.0f;
    static float pitch = 0.0f;
    static bool firstMouse = true;

    double xpos, ypos;
    glfwGetCursorPos(window_, &xpos, &ypos);

    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;

    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    glm::vec3 direction;
    direction.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    direction.y = sin(glm::radians(pitch));
    direction.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    camera_front_ = glm::normalize(direction);
}

} // namespace VXZ 