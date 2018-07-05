find_package(Qt5OpenGL REQUIRED)

find_package(GLEW)
include_directories(GLEW_INCLUDE_DIRS)

include_directories(${PROJECT_SOURCE_DIR}/geometry)
include_directories(${PROJECT_SOURCE_DIR}/scene_graph)

file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES *.h)

add_definitions(-DAPPNAME=\"${APPNAME}\" -DAPPVERSION=\"${APPVERSION}\")

if(NOT WIN32)
  add_definitions(-DGLX_GLXEXT_PROTOTYPES)
endif()

add_library(rendering SHARED ${SOURCE_FILES})

target_link_libraries(rendering stroke system Qt5::OpenGL ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES})
