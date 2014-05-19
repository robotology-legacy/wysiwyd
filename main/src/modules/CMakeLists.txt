# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#Add here modules without specific dependency
add_subdirectory(opcRelated)
add_subdirectory(motorSystem)
add_subdirectory(cvz)
add_subdirectory(tools)

if (WYSIWYD_USE_OpenCV)
	add_subdirectory(visualSystem)
endif()


