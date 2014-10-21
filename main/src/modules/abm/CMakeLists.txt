# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if(WYSIWYD_USE_PGSQL AND WYSIWYD_USE_OpenCV)
    add_subdirectory(autobiographicalMemory)
endif(WYSIWYD_USE_PGSQL AND WYSIWYD_USE_OpenCV)

add_subdirectory(abmHandler)
add_subdirectory(abmReasoning)
