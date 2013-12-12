find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(GLEW)
include_directories(GLEW_INCLUDE_DIRS)

find_package(QGLViewer)
include_directories(QGLVIEWER_INCLUDE_DIR)

find_package(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QTXML TRUE)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/app)

file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES *.h)
file(GLOB FORMS *.ui)
file(GLOB RESOURCES *.qrc) 

QT4_WRAP_CPP(HEADERS_MOC ${INCLUDE_FILES})
QT4_WRAP_UI(FORMS_HEADERS ${FORMS})
QT4_ADD_RESOURCES(RESOURCES_RCC ${RESOURCES})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

add_definitions(-DAPPNAME=\"${APPNAME}\" -DAPPVERSION=\"${APPVERSION}\")

add_library(app SHARED ${SOURCE_FILES} 
  ${HEADERS_MOC} ${FORMS_HEADERS} ${RESOURCES_RCC})

target_link_libraries(app view_map winged_edge geometry rendering scene_graph stroke image system
        ${QT_LIBRARIES} ${QGLVIEWER_LIBRARY_RELEASE} ${GLEW_LIBRARIES} ${PYTHON_LIBRARIES} ${OPENGL_LIBRARIES})

add_executable(freestyle ../main.cpp)

target_link_libraries(freestyle app)
