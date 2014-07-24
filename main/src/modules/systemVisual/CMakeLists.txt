# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(cvzVisualHierarchy)
#add_subdirectory(frontalEyeField)

find_package(kinectWrapper)

if(kinectWrapper_FOUND)
	add_subdirectory(agentDetector)
else()
	message("kinectWrapper has not been found, agentDetector will not be compiled")
endif()

if (WYSIWYD_USE_OpenCV)
	add_subdirectory(imageSplitter)
	add_subdirectory(imageFuser)
	add_subdirectory(caltechDatasetPlayer)
endif()


