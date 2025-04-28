cmake_minimum_required(VERSION 3.10)
project(voxelization_tests)

# Find required packages
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${GTEST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Add test executables
add_executable(voxel_grid_test core/voxel_grid_test.cpp)
add_executable(operator_test operator/operator_test.cpp)
add_executable(renderer_test renderer/renderer_test.cpp)
add_executable(storage_test storage/storage_test.cpp)
add_executable(voxelizer_test voxelizer/voxelizer_test.cpp)

# Link libraries
target_link_libraries(voxel_grid_test
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    pthread
)

target_link_libraries(operator_test
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    pthread
)

target_link_libraries(renderer_test
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    pthread
)

target_link_libraries(storage_test
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    pthread
)

target_link_libraries(voxelizer_test
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    pthread
)

# Add tests to CTest
add_test(NAME voxel_grid_test COMMAND voxel_grid_test)
add_test(NAME operator_test COMMAND operator_test)
add_test(NAME renderer_test COMMAND renderer_test)
add_test(NAME storage_test COMMAND storage_test)
add_test(NAME voxelizer_test COMMAND voxelizer_test) 