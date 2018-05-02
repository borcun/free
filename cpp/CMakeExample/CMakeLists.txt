cmake_minimum_required( VERSION 2.8.1 )
project(shape)

include(${CMAKE_CURRENT_SOURCE_DIR}/square/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/circle/CMakeLists.txt)

include_directories( "../square" "../circle" )

add_executable(shape main.cpp)
target_link_libraries(shape square circle)
