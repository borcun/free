cmake_minimum_required(VERSION 3.2)
project(mylib)
set(SRC mylib.c)
add_library(${PROJECT_NAME} SHARED ${SRC})
