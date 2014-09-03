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

#-------------------------------------------------------------------------------
include_directories(
    ${PROJECT_SOURCE_DIR}/opensubdiv
)

if( OPENGL_FOUND )
    include_directories(
        ${OPENGL_INCLUDE_DIR}
    )
    if (GLEW_FOUND)
        include_directories(
            ${GLEW_INCLUDE_DIR}
        )
    endif()
elseif( OPENGLES_FOUND )
    include_directories(
        ${OPENGLES_INCLUDE_DIR}
    )
endif()

#-------------------------------------------------------------------------------
# source & headers
set(CPU_SOURCE_FILES
    cpuKernel.cpp
    cpuComputeController.cpp
    cpuComputeContext.cpp
    cpuEvalLimitContext.cpp
    cpuEvalLimitController.cpp
    cpuEvalLimitKernel.cpp
    cpuEvalStencilsContext.cpp
    cpuEvalStencilsController.cpp
    cpuSmoothNormalContext.cpp
    cpuSmoothNormalController.cpp
    cpuVertexBuffer.cpp
    error.cpp
    evalLimitContext.cpp
    drawContext.cpp
    drawRegistry.cpp
)

set(GPU_SOURCE_FILES
)

set(KERNEL_FILES
)

set(INC_FILES
)

set(PRIVATE_HEADER_FILES
    debug.h
    cpuKernel.h
    cpuEvalLimitKernel.h
)

set(PUBLIC_HEADER_FILES
    computeController.h
    cpuComputeContext.h
    cpuComputeController.h
    cpuEvalLimitContext.h
    cpuEvalLimitController.h
    cpuEvalStencilsContext.h
    cpuEvalStencilsController.h
    cpuSmoothNormalContext.h
    cpuSmoothNormalController.h
    cpuVertexBuffer.h
    error.h
    evalLimitContext.h
    mesh.h
    nonCopyable.h
    opengl.h
    drawContext.h
    drawRegistry.h
    vertex.h
    vertexDescriptor.h
)

#-------------------------------------------------------------------------------
# platform dependent tweaks
if(APPLE OR UNIX)
    set(PLATFORM_COMPILE_FLAGS
        -fPIC
    )
elseif(WIN32)

endif()

add_definitions(
    ${PLATFORM_COMPILE_FLAGS}
)

set(DOXY_HEADER_FILES ${PUBLIC_HEADER_FILES})

#-------------------------------------------------------------------------------
set(OPENMP_PUBLIC_HEADERS
    ompKernel.h
    ompComputeController.h
    ompEvalStencilsController.h
    ompSmoothNormalController.h
)

if( OPENMP_FOUND )
    list(APPEND CPU_SOURCE_FILES
        ompKernel.cpp
        ompComputeController.cpp
        ompEvalStencilsController.cpp
        ompSmoothNormalController.cpp
    )

    list(APPEND PUBLIC_HEADER_FILES ${OPENMP_PUBLIC_HEADERS})

    if (CMAKE_COMPILER_IS_GNUCXX)
        list(APPEND PLATFORM_CPU_LIBRARIES
            gomp
        )
    endif()
endif()

list(APPEND DOXY_HEADER_FILES ${OPENMP_PUBLIC_HEADERS})

#-------------------------------------------------------------------------------
if( TBB_FOUND )
    include_directories(
        ${TBB_INCLUDE_DIR}
    )
    list(APPEND CPU_SOURCE_FILES
        tbbKernel.cpp
        tbbComputeController.cpp
        tbbEvalStencilsController.cpp
    )
    list(APPEND PUBLIC_HEADER_FILES
        tbbKernel.h
        tbbComputeController.h
        tbbEvalStencilsController.h
    )
    if (CMAKE_COMPILER_IS_GNUCXX)
        list(APPEND PLATFORM_CPU_LIBRARIES
            gomp
        )
    endif()

    list(APPEND PLATFORM_CPU_LIBRARIES
        ${TBB_LIBRARIES}
    )
endif()

#-------------------------------------------------------------------------------
set(GCD_PUBLIC_HEADERS
    gcdKernel.h
    gcdComputeController.h
)

if( GCD_FOUND )
    list(APPEND CPU_SOURCE_FILES
        gcdKernel.cpp
        gcdComputeController.cpp
    )
    list(APPEND PUBLIC_HEADER_FILES ${GCD_PUBLIC_HEADERS})
endif()

list(APPEND DOXY_HEADER_FILES ${GCD_PUBLIC_HEADERS})

#-------------------------------------------------------------------------------
# GL code & dependencies
set(GL_PUBLIC_HEADERS
    cpuGLVertexBuffer.h
    glDrawContext.h
    glDrawRegistry.h
    glVertexBuffer.h
    glMesh.h
)

list(APPEND DOXY_HEADER_FILES ${GL_PUBLIC_HEADERS})

#-------------------------------------------------------------------------------

if (NOT NO_LIB)

# Compile objs first for both the CPU and GPU libs -----
    add_library(osd_cpu_obj
        OBJECT
            ${CPU_SOURCE_FILES}
            ${PRIVATE_HEADER_FILES}
            ${PUBLIC_HEADER_FILES}
    )

    # Build static libs ------------------------------------
    add_library(osd_static_cpu
        STATIC
        $<TARGET_OBJECTS:osd_cpu_obj>
    )
    set_target_properties(osd_static_cpu PROPERTIES OUTPUT_NAME osdCPU CLEAN_DIRECT_OUTPUT 1)

    target_link_libraries(osd_static_cpu
        ${PLATFORM_CPU_LIBRARIES}
    )

    install( TARGETS osd_static_cpu DESTINATION "${CMAKE_LIBDIR_BASE}" )

    if (NOT WIN32 AND NOT IOS)

        # Evaluate 'soname' from OSD version
        # replace '_' with '.'
        string(REGEX REPLACE "(_)" "." OSD_SONAME ${OpenSubdiv_VERSION})

        # remove starting 'v' character
        string(REGEX REPLACE "^v" "" OSD_SONAME ${OSD_SONAME})

        # generate dynamic-link targets
        add_library(osd_dynamic_cpu
            SHARED
            $<TARGET_OBJECTS:osd_cpu_obj>
        )

        if (NOT ANDROID)
            set_target_properties(osd_dynamic_cpu
                PROPERTIES
                    OUTPUT_NAME osdCPU
                    CLEAN_DIRECT_OUTPUT 1
                    SOVERSION ${OSD_SONAME}
                )
        else()
            set_target_properties(osd_dynamic_cpu
                PROPERTIES
                    OUTPUT_NAME osdCPU
                    CLEAN_DIRECT_OUTPUT 1
                )
        endif()

        target_link_libraries(osd_dynamic_cpu
            ${PLATFORM_CPU_LIBRARIES}
        )

        install( TARGETS osd_dynamic_cpu LIBRARY DESTINATION "${CMAKE_LIBDIR_BASE}" )
    endif()
endif()

_add_doxy_headers( "${DOXY_HEADER_FILES}" )

install( FILES ${PUBLIC_HEADER_FILES}
         DESTINATION ${CMAKE_INCDIR_BASE}/osd
         PERMISSIONS OWNER_READ GROUP_READ WORLD_READ )

#-------------------------------------------------------------------------------
