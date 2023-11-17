cmake_minimum_required(VERSION 3.5)
project(thread-safe-function)
add_executable(${PROJECT_NAME} main.c)
target_link_libraries(${PROJECT_NAME} pthread)
