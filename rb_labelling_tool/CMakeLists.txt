cmake_minimum_required(VERSION 3.19.0)
project(rb_labelling_tool VERSION 0.1.0 LANGUAGES C CXX)

add_executable(rb_labelling_tool main.cpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(rb_labelling_tool pthread ${OpenCV_LIBS})

include(CTest)
enable_testing()
