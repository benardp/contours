find_package(Qt5Core REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES *.h)

add_definitions(-DAPPNAME=\"${APPNAME}\" -DAPPVERSION=\"${APPVERSION}\")

add_library(scene_graph SHARED ${SOURCE_FILES})

target_link_libraries(scene_graph Qt5::Core)