#include <iostream>
#include <random>
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "core/voxel_grid.hpp"
#include "voxelizer/box_voxelizer.hpp"
// #include "voxelizer/sphere_voxelizer.hpp"
// #include "voxelizer/corridor_voxelizer.hpp"
#include "storage/svo.hpp"
#include "renderer/voxel_renderer.hpp"
#include <glm/trigonometric.hpp>

using namespace VXZ;


// 随机数生成器
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<float> dis(0.0f, 1.0f);

// 生成随机位置
Eigen::Vector3f randomPosition(float min, float max) {
    return Eigen::Vector3f(
        min + dis(gen) * (max - min),
        min + dis(gen) * (max - min),
        min + dis(gen) * (max - min)
    );
}

// Camera control variables
float yaw = -90.0f;
float pitch = 0.0f;
float lastX = 640.0f;
float lastY = 360.0f;
bool firstMouse = true;
float cameraSpeed = 5.0f;
Eigen::Vector3f cameraFront(0.0f, 0.0f, -1.0f);
Eigen::Vector3f cameraRight(1.0f, 0.0f, 0.0f);
bool rotateCamera = true;

// Mouse callback function
void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
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

    Eigen::Vector3f front;
    front.x() = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y() = sin(glm::radians(pitch));
    front.z() = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = front.normalized();
    cameraRight = cameraFront.cross(Eigen::Vector3f(0.0f, 1.0f, 0.0f)).normalized();
}

int main() {
    // 创建体素网格
    // const int grid_size = 1000;
    // 创建体素网格，分辨率为1，最小值为(0,0,0)，最大值为(1000,1000,500)
    float resolution = 1.0f;
    Eigen::Vector3f min_bounds(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f max_bounds(1000.0f, 1000.0f, 500.0f);
    VoxelGrid grid(resolution, min_bounds, max_bounds);
    // VoxelGrid grid(grid_size, grid_size, grid_size);

    // 创建各种体素化器
  

    // 创建随机形状
    // 1. 立方体
    Eigen::Vector3f box_pos = randomPosition(100, 900);
    float box_size = 50.0f + dis(gen) * 100.0f;
    auto box_voxelizer = std::make_shared<BoxVoxelizerCPU>(box_pos, Eigen::Vector3f(box_size, box_size, box_size));      
    box_voxelizer->voxelize(grid);

    // // 2. 球体
    // Eigen::Vector3f sphere_pos = randomPosition(100, 900);
    // float sphere_radius = 30.0f + dis(gen) * 70.0f;
    // sphere_voxelizer->set_sphere(sphere_pos, sphere_radius);
    // sphere_voxelizer->voxelize(grid);

    // // 3. 飞行走廊
    // Eigen::Vector3f start_pos = randomPosition(100, 900);
    // Eigen::Vector3f end_pos = randomPosition(100, 900);
    // float corridor_width = 20.0f + dis(gen) * 30.0f;
    // float corridor_height = 20.0f + dis(gen) * 30.0f;
    // corridor_voxelizer->set_corridor(start_pos, end_pos, corridor_width, corridor_height);
    // corridor_voxelizer->voxelize(grid);

    // 使用 SVO 存储体素化结果
    auto svo = std::make_shared<SVOStorage>();
    svo->from_voxel_grid(grid);
    svo->save("svo.bin");
    
    // 创建渲染器并完成回调函数的设置
    VoxelRenderer renderer(1280, 720, "Voxelization Demo");

    // 初始化渲染器
    if (!renderer.initialize()) {
        std::cerr << "Failed to initialize renderer" << std::endl;
        return -1;
    }

    // 相机参数
    Eigen::Vector3f camera_pos(500.0f, 500.0f, 1000.0f);
    Eigen::Vector3f camera_target(500.0f, 500.0f, 0.0f);
    Eigen::Vector3f camera_up(0.0f, 1.0f, 0.0f);

    // Set up mouse callback and keyboard callback
    // glfwSetCursorPosCallback(renderer.get_window(), mouse_callback);  
    // glfwSetKeyCallback(renderer.get_window(), key_callback);


    // 渲染主循环
    while (!renderer.should_close()) {
        // 处理输入
        renderer.process_input();

        // 渲染
        renderer.render(grid);
    }

    return 0;
} 