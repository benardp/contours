include(FindPackageHandleStandardArgs)

if (WIN32)
    find_path( OSD_INCLUDE_DIR
        NAMES
            osd/mesh.h
        PATHS
            ${OSD_LOCATION}/include
            $ENV{OSD_LOCATION}/include
            $ENV{PROGRAMFILES}/osd/include
            ${PROJECT_SOURCE_DIR}/extern/osd/include
            DOC "The directory where osd/mesh.h resides" )

    find_library( OSD_LIBRARY
        NAMES
            osdCPU osdGPU
        PATHS
            ${OSD_LOCATION}/lib
            $ENV{OSD_LOCATION}/lib
            $ENV{PROGRAMFILES}/osd/lib
            ${PROJECT_SOURCE_DIR}/extern/osd/bin
            ${PROJECT_SOURCE_DIR}/extern/osd/lib
            DOC "The OSD library")
endif ()

if (${CMAKE_HOST_UNIX})
    find_path( OSD_INCLUDE_DIR
        NAMES
            osd/mesh.h
        PATHS
            ${OSD_LOCATION}/include
            $ENV{OSD_LOCATION}/include
            /usr/include
            /usr/local/include
            /usr/local/include/opensubdiv
            /sw/include
            /opt/local/include
            NO_DEFAULT_PATH
            DOC "The directory where osd/mesh.h resides"
    )
    find_library( OSD_LIBRARY
        NAMES
            osdCPU osdGPU
        PATHS
            ${OSD_LOCATION}/lib
            $ENV{OSD_LOCATION}/lib
            /usr/lib64
            /usr/lib
            /usr/local/lib64
            /usr/local/lib
            /sw/lib
            /opt/local/lib
            NO_DEFAULT_PATH
            DOC "The OSD library")
endif ()

find_package_handle_standard_args(OSD DEFAULT_MSG
    OSD_INCLUDE_DIR
    OSD_LIBRARY
)

mark_as_advanced( OSD_FOUND )
