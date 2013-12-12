include(FindPackageHandleStandardArgs)

if (WIN32)
    find_path( HBR_INCLUDE_DIR
        NAMES
            hbr/mesh.h
        PATHS
            ${HBR_LOCATION}/include
            $ENV{HBR_LOCATION}/include
            $ENV{PROGRAMFILES}/hbr/include
            ${PROJECT_SOURCE_DIR}/extern/hbr/include
            DOC "The directory where hbr/mesh.h resides" )
endif ()

if (${CMAKE_HOST_UNIX})
    find_path( HBR_INCLUDE_DIR
        NAMES
            hbr/mesh.h
        PATHS
            ${HBR_LOCATION}/include
            $ENV{HBR_LOCATION}/include
            /usr/include
            /usr/local/include
            /usr/local/include/opensubdiv
            /sw/include
            /opt/local/include
            NO_DEFAULT_PATH
            DOC "The directory where hbr/mesh.h resides"
    )
endif ()

find_package_handle_standard_args(HBR DEFAULT_MSG
    HBR_INCLUDE_DIR
)

mark_as_advanced( HBR_FOUND )
