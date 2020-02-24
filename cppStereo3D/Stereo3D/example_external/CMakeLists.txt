cmake_minimum_required(VERSION 3.5)

project(example_external VERSION 0.0.1 LANGUAGES CXX)

find_package(stereo3d CONFIG REQUIRED)

#include_directories(${stereo3d_SOURCE_DIR}/include/)

add_executable(example_external example.cpp)

target_link_libraries(example_external PUBLIC stereo3d::stereo3d)

if (WIN32)
# copy the .dll file to the same folder as the executable
add_custom_command(
    TARGET example_external POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    $<TARGET_FILE_DIR:stereo3d::stereo3d>
    $<TARGET_FILE_DIR:example_external>
    VERBATIM)
endif()