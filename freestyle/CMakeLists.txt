project(Freestyle)

cmake_minimum_required(VERSION 2.8.6)

set(APPNAME Freestyle)
set(APPVERSION_MAJ 2)
set(APPVERSION_MID 2)
set(APPVERSION_MIN 0)
set(APPVERSION ${APPVERSION_MAJ}.${APPVERSION_MID}.${APPVERSION_MIN})

set(EXECUTABLE_OUTPUT_PATH
    ${PROJECT_BINARY_DIR}
)

set(LIBRARY_OUTPUT_PATH
    ${PROJECT_BINARY_DIR}/lib
)

if(APPLE)
 set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")
endif()

add_subdirectory(geometry)

add_subdirectory(image)

add_subdirectory(scene_graph)

add_subdirectory(stroke)

add_subdirectory(system)

add_subdirectory(rendering)

add_subdirectory(view_map)

add_subdirectory(winged_edge)

add_subdirectory(swig)

add_subdirectory(app)

