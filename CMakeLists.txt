cmake_minimum_required(VERSION 3.15)
project(prc_nlopt)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)


add_executable(example src/main.cpp include/nmpc_controller.h)
add_executable(example2 src/sqp_example.cpp)
add_executable(nmpc src/nmpc_controller.cpp src/vehicle_model.cpp)
target_link_libraries(example "/usr/local/lib/libnlopt.so")
target_link_libraries(example2 "/usr/local/lib/libnlopt.so")
target_link_libraries(nmpc "/usr/local/lib/libnlopt.so")
