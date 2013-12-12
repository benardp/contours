find_package(Qt4 REQUIRED)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR})

file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES *.h)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

add_definitions(-DAPPNAME=\"${APPNAME}\" -DAPPVERSION=\"${APPVERSION}\")

add_library(system SHARED ${SOURCE_FILES})

target_link_libraries(system ${QT_LIBRARIES} ${PYTHON_LIBRARIES})