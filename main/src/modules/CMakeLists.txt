# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#Add here modules without specific dependency
add_subdirectory(slidingController)
add_subdirectory(cvz)
add_subdirectory(opcManager)
add_subdirectory(abmHandler)
add_subdirectory(abmReasoning)

if (WYSIWYD_USE_OpenCV)
	add_subdirectory(imageSplitter)
	add_subdirectory(imageFuser)
endif()


