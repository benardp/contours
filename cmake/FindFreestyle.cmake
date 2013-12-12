set(Freestyle_FLAVOR $ENV{FREESTYLE_FLAVOR})

if(UNIX)
        set(Freestyle_DIR "$ENV{FREESTYLE_DIR}/build/linux-g++-64/$ENV{FREESTYLE_FLAVOR}/lib")
endif(UNIX)
if(APPLE)
        set(Freestyle_DIR "$ENV{FREESTYLE_DIR}/build/macosx/$ENV{FREESTYLE_FLAVOR}/lib")
endif(APPLE)

if(${Freestyle_FLAVOR} MATCHES "debug")
    set(Freestyle_LIB "FreestyleAppLib_d")
else()
    set(Freestyle_LIB "FreestyleAppLib")
endif()


find_library(Freestyle_LIBRARY
        NAME
                ${Freestyle_LIB}
        PATHS
                ${Freestyle_DIR}
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Freestyle DEFAULT_MSG
    Freestyle_LIBRARY
)

mark_as_advanced( Freestyle_LIBRARY )
