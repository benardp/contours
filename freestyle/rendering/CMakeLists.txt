find_package(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL TRUE)

find_package(GLEW)
include_directories(GLEW_INCLUDE_DIRS)

find_package(OpenGL REQUIRED)
include_directories(OPENGL_INCLUDE_DIRS)

include_directories(${PROJECT_SOURCE_DIR}/geometry)
include_directories(${PROJECT_SOURCE_DIR}/scene_graph)

file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES *.h)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

add_definitions(-DAPPNAME=\"${APPNAME}\" -DAPPVERSION=\"${APPVERSION}\")

if(NOT WIN32)
  add_definitions(-DGLX_GLXEXT_PROTOTYPES)
endif()

add_library(rendering SHARED ${SOURCE_FILES})

target_link_libraries(rendering stroke system ${QT_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES})
