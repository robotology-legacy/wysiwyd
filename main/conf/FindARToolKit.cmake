# Copyright: (C) 2011 EFAA Consortium
# Author: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0
#
# - Try to find ARToolKit
# Once done, this will define
#
#  ARToolKit_FOUND - system has ARToolKit
#  ARToolKit_INCLUDE_DIRS - the ARToolKit include directories
#  ARToolKit_LIBRARIES - link these to use ARToolKit

SET (ARToolKit_FOUND TRUE)

IF (NOT ARToolKit_ROOT)
	SET (ARToolKit_ENV_ROOT $ENV{ARToolKit_ROOT})
	IF (ARToolKit_ENV_ROOT)
		SET(ARToolKit_ROOT ${ARToolKit_ENV_ROOT})
	ELSE (ARToolKit_ENV_ROOT)
		FIND_PATH(ARToolKit_ROOT /include/AR/ar.h ${CMAKE_PROJECT_DIR})
	ENDIF (ARToolKit_ENV_ROOT)
ENDIF (NOT ARToolKit_ROOT)

IF (NOT ARToolKit_ROOT)
	SET(ARToolKit_FOUND FALSE)
ENDIF(NOT ARToolKit_ROOT)

find_package(GLUT)

find_path(ARToolKit_INCLUDE_DIR /AR/ar.h ${ARToolKit_ROOT}/include)

message (STATUS ${ARToolKit_INCLUDE_DIR})

# Find the libraries
find_library(ARToolKit_LIBRARY
  NAMES libAR.a libAR
  PATHS ${ARToolKit_ROOT}/lib ${CMAKE_PROJECT_DIR}
)

find_library(ARToolKit_Video_LIBRARY
  NAMES libARvideo.a libARvideo
  PATHS ${ARToolKit_ROOT}/lib ${CMAKE_PROJECT_DIR}
)

find_library(ARToolKit_GSub_LIBRARY
  NAMES libARgsub.a libARgsub
  PATHS ${ARToolKit_ROOT}/lib ${CMAKE_PROJECT_DIR}
)

set(ARToolKit_PROCESS_INCLUDES ARToolKit_INCLUDE_DIR)
set(ARToolKit_PROCESS_LIBS ARToolKit_LIBRARY ARToolKit_Video_LIBRARY ARToolKit_GSub_LIBRARY)

SET (ARToolKit_INCLUDE_DIRS "")
SET (ARToolKit_LIBRARIES "")

foreach (i ${ARToolKit_PROCESS_INCLUDES})
  if (${i})
	set (ARToolKit_INCLUDE_DIRS ${ARToolKit_INCLUDE_DIRS} ${${i}})
	mark_as_advanced(${i})
  else (${i})
	set (ARToolKit_FOUND FALSE)
  endif (${i})
endforeach (i)

foreach (i ${ARToolKit_PROCESS_LIBS})
  if (${i})
	set (ARToolKit_LIBRARIES ${ARToolKit_LIBRARIES} ${${i}})
	mark_as_advanced(${i})
  else (${i})
	set (ARToolKit_FOUND FALSE)
  endif (${i})
endforeach (i)

if (ARToolKit_FOUND)
    message (STATUS "Found ARToolKit")
elseif (ARToolKit_FOUND)
	message (STATUS "ARToolKit not found")
endif (ARToolKit_FOUND)
	