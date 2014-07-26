# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#Add here modules without specific dependency
add_subdirectory(opcRelated)
add_subdirectory(cvz)
add_subdirectory(abm)
add_subdirectory(tools)
add_subdirectory(reservoirHandler)
add_subdirectory(navigationRelated)
add_subdirectory(reactiveLayer)
add_subdirectory(adaptiveLayer)
add_subdirectory(attentionRelated)
add_subdirectory(qRM)

add_subdirectory(systemMotor)
add_subdirectory(systemHaptic)

add_subdirectory(humanUnderstanding)

if(WYSIWYD_USE_OpenCV)
	add_subdirectory(systemVisual)
endif()
if(WYSIWYD_USE_TUIO)
	add_subdirectory(reactableRelated)
endif()
if(WYSIWYD_USE_OTL)
	add_subdirectory(bodySchema)
endif()

