# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Luke Boorman, Uriel Martinez
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if (WYSIWYD_USE_OpenCV AND WYSIWYD_USE_GPU)
    add_subdirectory(visionDriver)
endif()
add_subdirectory(speechInteraction)
add_subdirectory(abmActions)
