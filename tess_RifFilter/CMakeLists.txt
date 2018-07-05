project(tess_RifFilter)

cmake_minimum_required(VERSION 2.8.6)

add_subdirectory(opensubdiv)

include_directories(opensubdiv)
if(APPLE)
	set(LIBS ${LIBS} -Lopensubdiv/lib/ osdCPU osdutil)
else()
	set(LIBS ${LIBS} -Wl,-whole-archive osd_static_cpu osdutil -Wl,-no-whole-archive)
endif()
# find_package(3Delight REQUIRED)
# include_directories(${3Delight_INCLUDE_DIR})
# set(LIBS ${LIBS} ${3Delight_LIBRARIES})

# find_package(OSD REQUIRED) 
# include_directories(${OSD_INCLUDE_DIR})

# find_package(HBR REQUIRED)
# message(${HBR_INCLUDE_DIR})
# include_directories(${HBR_INCLUDE_DIR})

find_package(PRMAN REQUIRED)
include_directories(${PRMAN_INCLUDE_DIR})
set(LIBS ${LIBS} ${PRMAN_LIBRARIES})

set(LIBS ${LIBS} app)
add_definitions(-DLINK_FREESTYLE -DRENDERMAN)

set(SOURCE_FILES
  cameraModel.cpp
  refineContour.cpp
  vdtess.cpp
  subdiv.cpp
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
  VecMat.h
  subdiv.h
	rib2mesh.h
)

# if(APPLE OR UNIX)
#     set(PLATFORM_COMPILE_FLAGS
#         -fPIC
#     )
# endif()

# set(LIBRARY_OUTPUT_PATH
#     ${PROJECT_BINARY_DIR}/lib
# )

add_library(tess_RifFilter SHARED
	${SOURCE_FILES}
	${HEADER_FILES}
)

target_link_libraries(tess_RifFilter ${LIBS})
