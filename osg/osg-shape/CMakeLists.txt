cmake_minimum_required(VERSION 3.20)
project(osg-geo DESCRIPTION "OSG Geometry Example")

find_package(OpenThreads)
find_package(osg)
find_package(osgDB)
find_package(osgUtil)
find_package(osgViewer)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} OpenThreads osg osgDB osgUtil osgViewer)
