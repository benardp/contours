project(tess_RifFilter)

cmake_minimum_required(VERSION 2.8.6)

add_subdirectory(opensubdiv)

include_directories(opensubdiv)
set(LIBS ${LIBS} osd_static_cpu osdutil)

find_package(3Delight REQUIRED)
include_directories(${3Delight_INCLUDE_DIR})
set(LIBS ${LIBS} ${3Delight_LIBRARIES})

set(LIBS ${LIBS} app)
add_definitions(-DLINK_FREESTYLE)

set(SOURCE_FILES
        cameraModel.cpp
        refineContour.cpp
        subdiv.cpp
        vdtess.cpp
	rib2mesh.cpp
)

set(HEADER_FILES
	cameraModel.h
	chart.h
	chartFunctions.h
	meshFunctions.h
	paramPoint.h
	refineContour.h
	refineContourFunctions.h
        subdiv.h
        VecMat.h
	rib2mesh.h
)

if(APPLE OR UNIX)
    set(PLATFORM_COMPILE_FLAGS
        -fPIC
    )
endif()

set(LIBRARY_OUTPUT_PATH
    ${PROJECT_BINARY_DIR}/lib
)

add_library(tess_RifFilter SHARED
	${SOURCE_FILES}
	${HEADER_FILES}
)

target_link_libraries(tess_RifFilter ${LIBS})

