find_package(Qt5Widgets REQUIRED)

find_package(OpenGL REQUIRED)
include_directories(OPENGL_INCLUDE_DIRS)

include_directories(${PROJECT_SOURCE_DIR})

file(GLOB SOURCE_FILES *.cpp *.c)
file(GLOB INCLUDE_FILES *.h)

add_definitions(-DAPPNAME=\"${APPNAME}\" -DAPPVERSION=\"${APPVERSION}\")

add_library(view_map SHARED ${SOURCE_FILES})

target_link_libraries(view_map scene_graph winged_edge image system Qt5::Widgets ${OPENGL_LIBRARIES})
