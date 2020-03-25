cmake_minimum_required(VERSION 3.15)
project(prc_nlopt)

set(CMAKE_CXX_STANDARD 14)


add_executable(example main.cpp)
add_executable(example2 sqp_example.cpp)
target_link_libraries(example "/usr/local/lib/libnlopt.so")
target_link_libraries(example2 "/usr/local/lib/libnlopt.so")
