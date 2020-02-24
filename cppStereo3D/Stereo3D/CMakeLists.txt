cmake_minimum_required(VERSION 3.5)
project(stereo3d VERSION 1.0.0 LANGUAGES CXX)

option(ENABLE_EXAMPLES "ENABLE_TESTS" OFF)

find_package( OpenCV REQUIRED )

add_library(stereo3d SHARED
  src/Stereo3D.cpp
)

include(GNUInstallDirs)

target_include_directories(
  stereo3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries( stereo3d ${OpenCV_LIBS} )

target_compile_definitions(
  stereo3d PRIVATE -D_EXPORT
)

install(
    TARGETS stereo3d
    EXPORT stereo3d-config
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    EXPORT stereo3d-config
    NAMESPACE stereo3d::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/stereo3d)

install(
    DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/stereo3d)

if (ENABLE_EXAMPLES)

  add_executable(example_internal example_internal/example.cpp)
  target_link_libraries( example_internal PUBLIC stereo3d)

  install(TARGETS example_internal DESTINATION bin)

endif ()

# uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

if (UNIX)
  message("Building installer...")
  # debian installer
  SET(CPACK_GENERATOR "DEB")
  SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "I3DR")
  SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Library for capturing stereo images and generating 3D")
  INCLUDE(CPack)
endif (UNIX)