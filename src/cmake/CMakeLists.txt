cmake_minimum_required(VERSION 3.10)

project(arm_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wl,-no-as-needed")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/src/cmake")

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Include project headers
include_directories(
    ../../include
)

add_executable(arm ../arm.cpp)
target_link_libraries(arm m mujoco glfw)