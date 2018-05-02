cmake_minimum_required(VERSION 2.8.1)
project(square)

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build Type" FORCE)

include(GNUInstallDirs)
include_directories("inc")

add_library(square SHARED src/square.cpp)

target_include_directories(square PRIVATE inc)
target_include_directories(square PRIVATE src)
set_target_properties(square PROPERTIES VERSION 1 SOVERSION 1 PUBLIC_HEADER inc/square.h)

install(TARGETS square
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
