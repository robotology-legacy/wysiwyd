# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(kinectWrapper)

if(kinectWrapper_FOUND)
	add_subdirectory(agentDetector)
else()
	message("kinectWrapper has not been found, agentDetector will not be compiled")
endif()



