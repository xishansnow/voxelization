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


#CMAKE_PREFIX_PATH 用于指定CMake查找库的路径，供 find_package, find_program, find_library, find_file 和 find_path 使用，初始为空
list(APPEND CMAKE_PREFIX_PATH "/usr/share/eigen3/cmake")  # Eigen3， 线性代数库
list(APPEND CMAKE_PREFIX_PATH  "/usr/lib/cmake/glfw3") # GLFW, 窗口管理库
list(APPEND CMAKE_PREFIX_PATH "/usr/share/cmake/glm") # GLM, 图形学所需数学库
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/cmake/glew") # GLEW，OpenGL扩展库
list(APPEND CMAKE_PREFIX_PATH "/usr/share/octomap/") # Octomap, 3D体素库

#CMAKE_MODULE_PATH 用于指定CMake查找和包含模块的路径，供 include 或 find_package 使用。初始为空
list(APPEND CMAKE_MODULE_PATH "/usr/lib/x86_64-linux-gnu/cmake/OpenVDB") # OpenVDB, 3D体素库


# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(OpenVDB REQUIRED)

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
    #================================================================
    # Core files
    #================================================================
    src/core/voxel_grid.cpp
    
    #================================================================
    # Voxelizer files
    #================================================================
    # Base Class
    src/voxelizer/voxelizer.cpp

    # line 
    src/voxelizer/line_voxelizer.cpp
    src/voxelizer/polyline_voxelizer.cpp
    src/voxelizer/spline_voxelizer.cpp
    
    # CSG solid objects
    src/voxelizer/box_voxelizer.cpp
    src/voxelizer/cylinder_voxelizer.cpp
    src/voxelizer/sphere_voxelizer.cpp
    # src/voxelizer/corridor_voxelizer.cpp

    # B-Rep solid objects with tri
    src/voxelizer/triangle_mesh_voxelizer.cpp
    src/voxelizer/SurfaceVoxelizer.cpp
    src/voxelizer/SchwarzSolidVoxelizer.cpp
    src/voxelizer/EisemannSolidVoxelizer.cpp
    # point cloud objects
    src/voxelizer/point_cloud_voxelizer.cpp
    
    # Implicit representation objects 
    src/voxelizer/LevelSetVoxelizer.cpp # Level Set 
    src/voxelizer/SDFVoxelizer.cpp # Signed Distance Field
    src/voxelizer/ImplicitSurfaceVoxelizer.cpp # Implicit Surface

    #================================================================   
    # Storage files
    #================================================================

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
  
    
)

# Header files
set(HEADERS
    #================================================================
    # Core files
    #================================================================
    include/core/voxel_grid.hpp

    
#================================================================
# Voxelizer classes files
#================================================================
    # Base Class
    include/voxelizer/voxelizer.hpp
    include/voxelizer/voxelizer_base.hpp

    # Line Objects
    include/voxelizer/line_voxelizer.hpp
    include/voxelizer/polyline_voxelizer.hpp
    include/voxelizer/spline_voxelizer.hpp

    # Geometry Objects
    include/voxelizer/box_voxelizer.hpp
    include/voxelizer/cylinder_voxelizer.hpp
    include/voxelizer/sphere_voxelizer.hpp
    # include/voxelizer/corridor_voxelizer.hpp 

    # Surface Objects    
    include/voxelizer/triangle_mesh_voxelizer.hpp
    include/voxelizer/SurfaceVoxelizer.hpp
    include/voxelizer/SchwarzSolidVoxelizer.hpp
    include/voxelizer/EisemannSolidVoxelizer.hpp

    # Point Cloud Objects
    include/voxelizer/point_cloud_voxelizer.hpp

    # Level Set Objects
    include/voxelizer/LevelSetVoxelizer.hpp
    include/voxelizer/SDFVoxelizer.hpp
    include/voxelizer/ImplicitSurfaceVoxelizer.hpp


    # Multi-Level Voxel Grid
    # include/voxelizer/multi_level_voxel_grid.hpp

#================================================================
# Storage Classes files
#================================================================

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
    include/operator/grid_operator.hpp
    include/operator/binary_operator.hpp
    include/operator/intersection_operator.hpp
    include/operator/union_operator.hpp    

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
        tests/core/voxel_grid_test.cpp        
        tests/voxelizer_new_test.cpp
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
