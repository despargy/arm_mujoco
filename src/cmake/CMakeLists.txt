cmake_minimum_required(VERSION 3.10)

project(arm_mujoco)
add_compile_options(-std=c++11 -lorocos-kdl)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wl,-no-as-needed")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/src/cmake")

# You may need to adjust the path to Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} -I/usr/include/eigen3)

# Include other dependencies
# Include directories
find_path(NLOPT_INCLUDE_DIRS
    NAMES nlopt.h
    HINTS ${PC_NLOPT_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
find_library(NLOPT_LIBRARIES
    NAMES nlopt nlopt_cxx
    HINTS ${PC_NLOPT_LIBDIR})


include_directories(
    ../../include
    ${NLOPT_INCLUDE_DIRS}
)

add_executable(arm ../arm.cpp  ) 
target_link_libraries(arm ${NLOPT_LIBRARIES}  m mujoco glfw)
