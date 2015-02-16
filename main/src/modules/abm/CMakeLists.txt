# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if(WYSIWYD_USE_PGSQL AND WYSIWYD_USE_OpenCV AND WYSIWYD_USE_ACE)
    add_subdirectory(autobiographicalMemory)
endif(WYSIWYD_USE_PGSQL AND WYSIWYD_USE_OpenCV AND WYSIWYD_USE_ACE)

add_subdirectory(abmHandler)
add_subdirectory(abmReasoning)
