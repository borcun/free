# CMake Version
cmake_minimum_required(VERSION 2.8)
# Project Name
project(Pipemill)

include_directories(${DIR}/headers /usr/local/include)

# Add Project Source File
add_executable(Pipemill BaseFilter.hpp PipemillTest.hpp InConnector.cpp OutConnector.cpp InputFilter.cpp ProcessFilter.cpp OutputFilter.cpp Pipeline.cpp MultiPipeline.cpp main.cpp)
# find OpenCV and Boost package
find_package( OpenCV REQUIRED COMPONENTS core highgui imgproc )
find_package( Boost REQUIRED COMPONENTS thread system )
# link target libraries
target_link_libraries(Pipemill tbb rt pthread ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
