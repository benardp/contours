project(Contours)

cmake_minimum_required(VERSION 2.8.6)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebug")
endif()

add_subdirectory(freestyle)

add_subdirectory(tess_RifFilter)

file(COPY ${CMAKE_SOURCE_DIR}/scripts DESTINATION ${PROJECT_BINARY_DIR})
