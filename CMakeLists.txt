cmake_minimum_required(VERSION 3.10)
project(Voxelization VERSION 0.1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



# Options
option(BUILD_TESTS "Build tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(USE_OPENGL "Enable OpenGL support" ON)

set(OpenGL_GL_PREFERENCE GLVND)


list(APPEND CMAKE_PREFIX_PATH "/usr/share/eigen3/cmake")  # Eigen3， 线性代数库
list(APPEND CMAKE_PREFIX_PATH  "/usr/local/lib/cmake/glfw3") # GLFW, 窗口管理库
list(APPEND CMAKE_PREFIX_PATH "/usr/share/cmake/glm") # GLM, 图形学所需数学库
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/cmake/glew") # GLEW，OpenGL扩展库



# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# CCache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_CUDA_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

if(BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
    ${GLFW3_INCLUDE_DIR}
    ${GLM_INCLUDE_DIR}
)

# Source files
set(SOURCES
    # Core files
    src/core/voxel_grid.cpp

    # Voxelizer files
    src/voxelizer/voxelizer.cpp
    src/voxelizer/box_voxelizer.cpp
    src/voxelizer/cylinder_voxelizer.cpp
    src/voxelizer/sphere_voxelizer.cpp
    # src/voxelizer/corridor_voxelizer.cpp

    src/voxelizer/line_voxelizer.cpp
    src/voxelizer/polyline_voxelizer.cpp
    src/voxelizer/spline_voxelizer.cpp

    src/voxelizer/triangle_mesh_voxelizer.cpp
    src/voxelizer/point_cloud_voxelizer.cpp
    # src/voxelizer/multi_level_voxel_grid.cpp

    # Storage files
    src/storage/voxelstorage.cpp
    src/storage/svo.cpp
    # src/storage/svdag.cpp
    # src/storage/ssvdag.cpp
    # src/storage/openvdb_storage.cpp
    # src/storage/nanovdb_storage.cpp

    # Renderer files
    src/renderer/voxel_renderer.cpp
    src/renderer/volume_renderer.cpp

    # Operator files
    src/operator/SmoothOperator.cpp
    src/operator/DilateOperator.cpp
    src/operator/ErodeOperator.cpp
    src/operator/OffsetOperator.cpp
    src/operator/UnionOperator.cpp
    src/operator/IntersectionOperator.cpp
    src/operator/DifferenceOperator.cpp
    src/operator/OpeningOperator.cpp
    src/operator/ClosingOperator.cpp
    src/operator/DistanceTransformOperator.cpp
    src/operator/ConnectedComponentsOperator.cpp
    src/operator/FillOperator.cpp
    src/operator/InterpolationOperator.cpp
    # src/operator/grid_operator.cpp
)

# Header files
set(HEADERS
    # Core files
    include/core/voxel_grid.hpp

    # Voxelizer files
    include/voxelizer/voxelizer.hpp
    include/voxelizer/voxelizer_base.hpp


    # Voxelizer files
    include/voxelizer/box_voxelizer.hpp
    include/voxelizer/cylinder_voxelizer.hpp
    include/voxelizer/sphere_voxelizer.hpp
    # include/voxelizer/corridor_voxelizer.hpp

    include/voxelizer/line_voxelizer.hpp
    include/voxelizer/polyline_voxelizer.hpp
    include/voxelizer/spline_voxelizer.hpp

    include/voxelizer/point_cloud_voxelizer.hpp
        include/voxelizer/triangle_mesh_voxelizer.hpp
    # include/voxelizer/multi_level_voxel_grid.hpp

    # Storage files
    include/storage/voxelstorage.hpp
    include/storage/svo.hpp
    # include/storage/svdag.hpp
    # include/storage/ssvdag.hpp
    # include/storage/openvdb_storage.hpp
    # include/storage/nanovdb_storage.hpp

    # Renderer files
    include/renderer/voxel_renderer.hpp
    include/renderer/volume_renderer.hpp

    # Operator files
    include/operator/SmoothOperator.hpp
    include/operator/DilateOperator.hpp
    include/operator/ErodeOperator.hpp
    include/operator/OffsetOperator.hpp
    include/operator/UnionOperator.hpp
    include/operator/IntersectionOperator.hpp
    include/operator/DifferenceOperator.hpp
    include/operator/OpeningOperator.hpp
    include/operator/ClosingOperator.hpp
    include/operator/DistanceTransformOperator.hpp
    include/operator/ConnectedComponentsOperator.hpp
    include/operator/FillOperator.hpp
    include/operator/InterpolationOperator.hpp
    include/operator/grid_operator.hpp
)

# Create library
add_library(voxelizer STATIC ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(voxelizer
    ${EIGEN3_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${GLFW3_LIBRARIES}
    ${GLM_LIBRARIES}
)

# Install rules
install(TARGETS voxelizer
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# Tests
if(BUILD_TESTS)
  set(TEST_SOURCES
        tests/operator_test.cpp
        tests/storage_test.cpp
    )

  add_executable(voxelizer_tests ${TEST_SOURCES})
  target_link_libraries(voxelizer_tests
        voxelizer
        GTest::GTest
        GTest::Main
    )
  add_test(NAME voxelizer_tests COMMAND voxelizer_tests)
endif()

# Examples
if(BUILD_EXAMPLES)
  set(EXAMPLE_SOURCES
        examples/voxelization_demo.cpp
    )

  add_executable(voxelization_demo ${EXAMPLE_SOURCES})
  target_link_libraries(voxelization_demo
                        voxelizer
                        glfw
                        ${OPENGL_LIBRARIES}
                        ${GLEW_LIBRARIES}
                        ${GLM_LIBRARIES}
  )
endif()

# Export targets
export(TARGETS voxelizer FILE ${CMAKE_BINARY_DIR}/VoxelizationTargets.cmake)
