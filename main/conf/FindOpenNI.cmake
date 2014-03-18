# Copyright: (C) 2012 Miguel Sarabia del Castillo
# Imperial College London
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

# - Try to find OpenNI
# Once done this will define
#  OpenNI_FOUND - System has OpenNI
#  OpenNI_INCLUDE_DIRS - The OpenNI include directories
#  OpenNI_LIBRARIES - The libraries needed to use OpenNI
#  OpenNI_DIR - Directory where OpenNI was found (can be set by user to force 
#               CMake to look in a particular directory)


#Set hints
set(OpenNI_INCLUDE_HINTS
  "${OpenNI_DIR}/include"
  "/usr/local/include"
  "C:/Program Files/OpenNI/Include"
  "C:/Program Files (x86)/OpenNI/Include"
)

set(OpenNI_LIB_HINTS
  "${OpenNI_DIR}/lib"
  "${OpenNI_DIR}/Lib64"
  "/usr/local/lib"
  "/usr/local/lib64"
  "C:/Program Files/OpenNI/Lib64"
  "C:/Program Files/OpenNI/Lib"
  "C:/Program Files (x86)/OpenNI/Lib"
)

#Set Definitions
set(OpenNI_DEFINITIONS "")

#Find header files
find_path(OpenNI_INCLUDE_DIR XnOpenNI.h
          HINTS ${OpenNI_INCLUDE_HINTS}
          PATH_SUFFIXES ni )

#Find libraries
find_library(OpenNI_LIBRARY NAMES OpenNI
             HINTS ${OpenNI_LIB_HINTS})

#If library was not found, we may be using 64bit version on windows which has
#a different name
if ( OpenNI_LIBRARY STREQUAL "OpenNI_LIBRARY-NOTFOUND" )
  find_library(OpenNI_LIBRARY NAMES OpenNI64
               HINTS ${OpenNI_LIB_HINTS} )
endif()


set(OpenNI_LIBRARIES ${OpenNI_LIBRARY} )
set(OpenNI_INCLUDE_DIRS ${OpenNI_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OpenNI_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(OpenNI  DEFAULT_MSG
                                  OpenNI_LIBRARY OpenNI_INCLUDE_DIR)

set(OpenNI_FOUND ${OPENNI_FOUND})

if ( OpenNI_FOUND )
  # If openni was found check its root directory
  get_filename_component(OpenNI_NEW_DIR "${OpenNI_INCLUDE_DIRS}/../" ABSOLUTE)
endif()

# Allow the user to propose a hint about where to find OpenNI
# Override value with found directory (empty if not found)
set(OpenNI_DIR ${OpenNI_NEW_DIR} CACHE FILEPATH "OpenNI root directory" FORCE)

mark_as_advanced(OpenNI_INCLUDE_DIR OpenNI_LIBRARY OPENNI_FOUND )

