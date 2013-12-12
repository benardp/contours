find_package(Qt4 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

file(GLOB SOURCE_FILES *.cpp *.c)
file(GLOB INCLUDE_FILES *.h)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

add_definitions(-DAPPNAME=\"${APPNAME}\" -DAPPVERSION=\"${APPVERSION}\")

add_library(view_map SHARED ${SOURCE_FILES})

target_link_libraries(view_map ${QT_LIBRARIES})
