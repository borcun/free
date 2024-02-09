cmake_minimum_required(VERSION 3.2)
project(hello_world VERSION 1.2)
set(CMAKE_CXX_STANDARD 11)

configure_file(version.h.in version.h)
add_executable(hello_world MACOSX_BUNDLE main.cpp)
target_include_directories(hello_world PUBLIC "${PROJECT_BINARY_DIR}")
