# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#Add here modules without specific dependency
add_subdirectory(awareTouch)
add_subdirectory(opcRelated)
add_subdirectory(motorSystem)
add_subdirectory(cvz)
add_subdirectory(abm)
add_subdirectory(tools)
add_subdirectory(reservoirHandler)
add_subdirectory(interpersonalDistanceRegulator)
add_subdirectory(reactiveLayer)
add_subdirectory(adaptiveLayer)

if(WYSIWYD_USE_OpenCV)
	add_subdirectory(visualSystem)
endif()
