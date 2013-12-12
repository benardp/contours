include(FindPackageHandleStandardArgs)

if (WIN32)
    find_path( FAR_INCLUDE_DIR
        NAMES
            far/mesh.h
        PATHS
            ${FAR_LOCATION}/include
            $ENV{FAR_LOCATION}/include
            $ENV{PROGRAMFILES}/far/include
            ${PROJECT_SOURCE_DIR}/extern/far/include
            DOC "The directory where far/mesh.h resides" )
endif ()

if (${CMAKE_HOST_UNIX})
    find_path( FAR_INCLUDE_DIR
        NAMES
            far/mesh.h
        PATHS
            ${FAR_LOCATION}/include
            $ENV{FAR_LOCATION}/include
            /usr/include
            /usr/local/include
            /usr/local/include/opensubdiv
            /sw/include
            /opt/local/include
            NO_DEFAULT_PATH
            DOC "The directory where far/mesh.h resides"
    )
endif ()

find_package_handle_standard_args(FAR DEFAULT_MSG
    FAR_INCLUDE_DIR
)

mark_as_advanced( FAR_FOUND )
