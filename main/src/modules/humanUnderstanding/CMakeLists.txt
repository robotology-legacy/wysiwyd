# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(kinectWrapper QUIET)

if(kinectWrapper_FOUND)
	add_subdirectory(agentDetector)
	add_subdirectory(kinect2vector)
else()
	message(STATUS "kinectWrapper has not been found, some modules will not be compiled")
endif()



