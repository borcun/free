cmake_minimum_required(VERSION 2.8.1)
project(circle)

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build Type" FORCE)

include(GNUInstallDirs)
include_directories("inc")

add_library(circle SHARED src/circle.cpp)

target_include_directories(circle PRIVATE inc)
target_include_directories(circle PRIVATE src)
set_target_properties(circle PROPERTIES VERSION 1 SOVERSION 1 PUBLIC_HEADER inc/circle.h)

install(TARGETS circle
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
