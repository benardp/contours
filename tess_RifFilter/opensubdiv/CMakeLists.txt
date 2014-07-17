#
#   Copyright 2013 Pixar
#
#   Licensed under the Apache License, Version 2.0 (the "Apache License")
#   with the following modification; you may not use this file except in
#   compliance with the Apache License and the following modification to it:
#   Section 6. Trademarks. is deleted and replaced with:
#
#   6. Trademarks. This License does not grant permission to use the trade
#      names, trademarks, service marks, or product names of the Licensor
#      and its affiliates, except as required to comply with Section 4(c) of
#      the License and to reproduce the content of the NOTICE file.
#
#   You may obtain a copy of the Apache License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the Apache License with the above modification is
#   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#   KIND, either express or implied. See the Apache License for the specific
#   language governing permissions and limitations under the Apache License.
#

#
#   Copyright 2013 Pixar
#
#   Licensed under the Apache License, Version 2.0 (the "Apache License")
#   with the following modification; you may not use this file except in
#   compliance with the Apache License and the following modification to it:
#   Section 6. Trademarks. is deleted and replaced with:
#
#   6. Trademarks. This License does not grant permission to use the trade
#      names, trademarks, service marks, or product names of the Licensor
#      and its affiliates, except as required to comply with Section 4(c) of
#      the License and to reproduce the content of the NOTICE file.
#
#   You may obtain a copy of the Apache License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the Apache License with the above modification is
#   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#   KIND, either express or implied. See the Apache License for the specific
#   language governing permissions and limitations under the Apache License.
#

project(OpenSubdiv)

cmake_minimum_required(VERSION 2.8.6)

#-------------------------------------------------------------------------------
# Obtain OpenSubdiv API version from version.h file
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.h")
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version.h"
        OpenSubdiv_VERSION REGEX "^#define OPENSUBDIV_VERSION .*$")
    string(REPLACE "#define OPENSUBDIV_VERSION " "" OpenSubdiv_VERSION ${OpenSubdiv_VERSION})
else()
    message(FATAL_ERROR, "Cannot locate version.h in CMAKE_CURRENT_SOURCE_DIR")
    message(${CMAKE_CURRENT_SOURCE_DIR})
endif()

#-------------------------------------------------------------------------------

message(STATUS "Compiling ${CMAKE_PROJECT_NAME} version ${OpenSubdiv_VERSION}")
message(STATUS "Using cmake version ${CMAKE_VERSION}")

#-------------------------------------------------------------------------------
# Specify the default install path
if (NOT DEFINED CMAKE_INSTALL_PREFIX)
    SET( CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/ )
endif()

if (NOT DEFINED CMAKE_INCDIR_BASE)
    set( CMAKE_INCDIR_BASE include/opensubdiv )
endif()

if (NOT DEFINED CMAKE_BINDIR_BASE)
    set( CMAKE_BINDIR_BASE bin )
endif()

if (NOT DEFINED CMAKE_LIBDIR_BASE)
    set( CMAKE_LIBDIR_BASE lib )
endif()

if (NOT DEFINED CMAKE_PLUGINDIR_BASE)
    set( CMAKE_PLUGINDIR_BASE plugin )
endif()

if (NOT DEFINED CMAKE_DOCDIR_BASE)
    set( CMAKE_DOCDIR_BASE share/doc/opensubdiv )
else()
    if (IS_ABSOLUTE ${CMAKE_DOCDIR_BASE})
        set( CMAKE_DOCDIR_BASE ${CMAKE_DOCDIR_BASE} )
    else()
        set( CMAKE_DOCDIR_BASE ${CMAKE_INSTALL_PREFIX}/${CMAKE_DOCDIR_BASE} )
    endif()
endif()

# Allow install path to be overridden for cross-compile builds
if(LIBRARY_OUTPUT_PATH_ROOT)
    SET( CMAKE_INSTALL_PREFIX ${LIBRARY_OUTPUT_PATH_ROOT}/ )
endif()

# Set the directory where the executables will be stored.
set(EXECUTABLE_OUTPUT_PATH
    ${PROJECT_BINARY_DIR}/bin
)

# Set the directory where the libraries will be stored.
set(LIBRARY_OUTPUT_PATH
    ${PROJECT_BINARY_DIR}/lib
)

list(APPEND CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#-------------------------------------------------------------------------------
# Detect Clang (until a cmake version provides built-in variables)
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(CMAKE_COMPILER_IS_CLANGCC 1)
elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Intel")
    set(CMAKE_COMPILER_IS_ICC 1)
endif()


set(OSD_COMPILER_FLAGS)

# Disable spurrious warnings in gcc builds and clang
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_CLANGCC OR CMAKE_COMPILER_IS_ICC )

    # Turn on all warnings
    list(APPEND OSD_COMPILER_FLAGS -Wall)

    # HBR uses the offsetof macro on a templated struct, which appears
    # to spurriously set off this warning in both gccc and Clang
    list(APPEND OSD_COMPILER_FLAGS -Wno-invalid-offsetof)

    # HBR uses unions as an optimization for its memory allocation.
    # Type casting between union members breaks strict aliasing rules from
    # gcc 4.4.1 versions onwards. We disable the warning but keep aliasing
    # optimization.
    list(APPEND OSD_COMPILER_FLAGS -Wno-strict-aliasing)

    # FAR and OSD have templated virtual function implementations that trigger
    # a lot of hidden virtual function overloads (some of them spurrious).
    # Disable those for now in Clang.
    if(CMAKE_COMPILER_IS_CLANGCC)
        list(APPEND OSD_COMPILER_FLAGS -Wno-overloaded-virtual)
    endif()

    # Intel's icc compiler requires some libraries linked
    if(CMAKE_COMPILER_IS_ICC)

        foreach (ICC_LIB iomp5 irng intlc)

            if(CMAKE_SIZEOF_VOID_P MATCHES "8")
                list(APPEND ICC_LIB_ARCH "intel64")
            elseif(CMAKE_SIZEOF_VOID_P MATCHES "4")
                list(APPEND ICC_LIB_ARCH "ia32")
            endif()

            find_library( ICC_${ICC_LIB}
                NAMES
                    ${ICC_LIB}
                PATHS
                    ${ICC_LOCATION}/lib
                    /opt/intel/lib/
                PATH_SUFFIXES
                    ${ICC_LIB_ARCH}
            )

            if (ICC_${ICC_LIB})
                list(APPEND ICC_LIBRARIES ${ICC_${ICC_LIB}})
            else()
                message( FATAL_ERROR "${ICC_${ICC_LIB}} library not found - required by icc" )
            endif()

        endforeach()
    endif()

elseif(MSVC)

    # Turn on all warnings
    list(APPEND OSD_COMPILER_FLAGS /Wall)

    # MSVC is unfortunately not standard conforming with regards to
    # the alternative names for logical and bitwise operators:
    # http://stackoverflow.com/questions/555505/c-alternative-tokens
    # http://stackoverflow.com/questions/6006526/c-writing-or-instead-of
    #
    # This can be solved by including iso646.h, but that is a rather
    # unsatisfactory solution since we then always have to remember to
    # include this header file.  Instead we define these operators
    # ourselves as command line arguments to cl.exe.
    #
    # An alternative would be to compile with the /Za option
    # (but unfortunately that breaks other code):
    # http://msdn.microsoft.com/en-us/library/0k0w269d.aspx
    list(APPEND OSD_COMPILER_FLAGS
                    /Dand=&&
                    /Dand_eq=&=
                    /Dbitand=&
                    /Dbitor=|
                    /Dcompl=~
                    /Dnot=!
                    /Dnot_eq=!=
                    /Dor=||
                    /Dor_eq=|=
                    # nvcc does not seem to like a caret being the last character
                    # in a command line defined preprocessor symbol, so add an
                    # empty trailing comment to avoid this.
                    /Dxor=^/**/
                    /Dxor_eq=^=
    )


    list(APPEND OSD_COMPILER_FLAGS
                    /W3     # Use warning level recommended for production purposes.
                    /WX     # Treat all compiler warnings as errors.

                    # these warnings are being triggered from inside VC's header files
                    # warning C4350: behavior change: 'member1' called instead of 'member2'
                    /wd4350
                    # warning C4548: expression before comma has no effect; expected expression with side-effect
                    /wd4548

                    # Make sure WinDef.h does not define min and max macros which
                    # will conflict with std::min() and std::max().
                    /DNOMINMAX

                    # Make sure the constants in <math.h> get defined.
                    /D_USE_MATH_DEFINES

                    # Do not enforce MSVC's safe CRT replacements.
                    /D_CRT_SECURE_NO_WARNINGS

                    # Disable checked iterators and iterator debugging.  Visual Studio
                    # 2008 does not implement std::vector::data(), so we need to take the
                    # address of std::vector::operator[](0) to get the memory location of
                    # a vector's underlying data storage.  This does not work for an empty
                    # vector if checked iterators or iterator debugging is enabled.

                    # XXXX manuelk : we can't force SECURE_SCL to 0 or client code has
                    # problems linking against OSD if their build is not also
                    # overriding SSCL to the same value.
                    # See : http://msdn.microsoft.com/en-us/library/vstudio/hh697468.aspx
                    #/D_SECURE_SCL=0
                    #/D_HAS_ITERATOR_DEBUGGING=0
    )

    # Turn off a duplicate LIBCMT linker warning
    set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:libcmt.lib")
    set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:libcmt.lib")

endif()

if(${SIMD} MATCHES "AVX")
    list(APPEND OSD_COMPILER_FLAGS -xAVX)
endif()

add_definitions(${OSD_COMPILER_FLAGS})

#-------------------------------------------------------------------------------

# Ignore rules that will re-run cmake (this will avoid constant
# reloading of the generated Visual Studio project).
set(CMAKE_SUPPRESS_REGENERATION TRUE)

# Check for dependencies
if(NOT NO_OMP)
    find_package(OpenMP)
endif()
if(NOT NO_TBB)
    find_package(TBB 4.0)
endif()
find_package(OpenGL)
find_package(OpenGLES)
if(NOT ANDROID AND NOT IOS)
    find_package(GLFW 2.7.0)

endif()
find_package(Doxygen 1.8.4)
find_package(Docutils 0.6)
if (OPENGL_FOUND)
    if (APPLE)
        find_package(GLEW)
    else()
        find_package(GLEW REQUIRED)
    endif()
endif()

if (NOT NO_GCD AND APPLE)
    set(GCD_FOUND 1)
endif()

# Warn about missing dependencies that will cause parts of OpenSubdiv to be
# disabled.  Also, add preprocessor defines that can be used in the source
# code to determine if a specific dependency is present or not.

if(GCD_FOUND)
    add_definitions( -DOPENSUBDIV_HAS_GCD )
endif()

if(OPENMP_FOUND)
    add_definitions(
        -DOPENSUBDIV_HAS_OPENMP
        ${OpenMP_CXX_FLAGS}
    )
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
    message(WARNING
        "OpenMP was not found : support for OMP parallel compute kernels "
        "will be diabled in Osd.  If your compiler supports OpenMP "
        "directives, please refer to the FindOpenMP.cmake shared module "
        "in your cmake installation.")
endif()

if(TBB_FOUND)
    add_definitions(
        -DOPENSUBDIV_HAS_TBB
        ${TBB_CXX_FLAGS}
    )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TBB_CXX_FLAGS}")
else()
    message(WARNING
        "TBB was not found : support for TBB parallel compute kernels "
        "will be diabled in Osd.  If your compiler supports TBB "
        "directives, please refer to the FindTBB.cmake shared module "
        "in your cmake installation.")
endif()

if(GLFW_FOUND AND (GLFW_VERSION VERSION_EQUAL 3.0 OR GLFW_VERSION VERSION_GREATER 3.0))
    add_definitions( -DGLFW_VERSION_3 )
endif()

# note: perhaps rename this to be more consistent, e.g. OPENSUBDIV_USES_GLEW
if(GLEW_FOUND)
    add_definitions( -DOSD_USES_GLEW )
endif()

if(OPENGLES_FOUND)
    add_definitions(
        -DOPENSUBDIV_HAS_OPENGLES
    )
endif()

# Link examples & regressions dynamically against Osd
set( OSD_LINK_TARGET osd_dynamic_cpu )

if (WIN32)
    add_definitions(
        # Link against the static version of GLEW.
        -DGLEW_STATIC
    )
    # Link examples & regressions statically against Osd for
    # Windows until all the kinks can be worked out.
    set( OSD_LINK_TARGET osd_static_cpu )

endif()


#-------------------------------------------------------------------------------
# General-use macros

# Macro for processing public headers into the build area for doxygen processing

#set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_custom_target( public_headers )

macro(_add_doxy_headers headers)
    if (NOT NO_DOC AND DOXYGEN_FOUND)
        file(RELATIVE_PATH path ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )

        string(REPLACE  "/" "_" targetpath ${path})

        foreach (header ${headers})

            set(infile ${CMAKE_CURRENT_SOURCE_DIR}/${header})
            set(outfile ${CMAKE_BINARY_DIR}/public_headers/${path}/${header})
            set(targetname "${targetpath}_${header}")

            add_custom_command(
                OUTPUT
                    ${outfile}
                COMMAND
                    ${CMAKE_COMMAND}
                ARGS
                    -E copy ${infile} ${outfile}
                DEPENDS
                    ${infile}
            )

            add_custom_target(${targetname} DEPENDS ${outfile})

            #set_target_properties(${targetname} PROPERTIES FOLDER "public_headers/")

            list(APPEND headerfiles ${targetname} )
        endforeach()

        add_dependencies( public_headers DEPENDS ${headerfiles} )
    endif()
endmacro()


# Macro wrapper for adding a non-cuda dependent executable
macro(_add_executable target)

    add_executable(${target} ${ARGN})

    if(CMAKE_COMPILER_IS_ICC)
        target_link_libraries(${target} ${ICC_LIBRARIES})
    endif()
endmacro()

#-------------------------------------------------------------------------------
# Build targets

add_subdirectory(hbr)

add_subdirectory(far)

add_subdirectory(osd)

add_subdirectory(osdutil)
