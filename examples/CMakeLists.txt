cmake_minimum_required(VERSION 3.10)
project(cube_renderer)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Add executable
add_executable(cube_renderer cube_renderer.cpp)

# Link libraries
target_link_libraries(cube_renderer
    ${OPENGL_LIBRARIES}
    GLEW::GLEW
    glfw
    glm::glm
)

# Set include directories
target_include_directories(cube_renderer PRIVATE
    ${OPENGL_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
) 