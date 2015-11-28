# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Hyung Jin Chang
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if (WYSIWYD_USE_OpenCV)
    add_subdirectory(faceTracker)
#   add_subdirectory(kinematicStructure)
    add_subdirectory(headPoseEstimator)
endif()

if(WYSIWYD_USE_CURL)
    add_subdirectory(rekognition)
#    add_subdirectory(oxford)
    add_subdirectory(sightcorp)
endif()
