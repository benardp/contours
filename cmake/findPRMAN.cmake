#==========
#
# Copyright (c) 2010, Dan Bethell.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of Dan Bethell nor the names of any
#       other contributors to this software may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#==========
#
# Variables defined by this module:
#   PRMAN_FOUND
#   PRMAN_INCLUDE_DIR
#   PRMAN_COMPILE_FLAGS
#   PRMAN_LIBRARIES
#   PRMAN_LIBRARY_DIR
#
# Usage:
#   FIND_PACKAGE( PRMAN )
#   FIND_PACKAGE( PRMAN REQUIRED )
#
# Note:
# You can tell the module where PRMAN is installed by setting
# the PRMAN_LOCATION (or setting the RMANTREE environment
# variable) before calling FIND_PACKAGE.
#
# E.g.
#   SET( PRMAN_LOCATION "/opt/pixar/RenderManProServer-15.0" )
#   FIND_PACKAGE( PRMAN REQUIRED )
#
#==========

# Normalise the RMANTREE path to use forward slashes if it doesn't already
# get_filename_component(RMANTREE $ENV{RMANTREE} ABSOLUTE)

find_path( PRMAN_INCLUDE_DIR ri.h
  "${PRMAN_LOCATION}/include"
  "${RMANTREE}/include"
  )

# A hint for find_library for libprman
SET(CMAKE_FIND_LIBRARY_PREFIXES "lib")

find_library( PRMAN_LIBRARIES prman
  "${PRMAN_LOCATION}/lib"
  "${RMANTREE}/lib"
  )

# our compilation flags
SET( PRMAN_COMPILE_FLAGS "-DPRMAN -fPIC" )

# did we find everything?
include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( "PRMAN" DEFAULT_MSG
  PRMAN_INCLUDE_DIR
  PRMAN_LIBRARIES
  )