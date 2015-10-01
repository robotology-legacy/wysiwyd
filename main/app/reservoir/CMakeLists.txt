# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Grégoire Pointeau
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(bodyReservoir)

if(WYSIWYD_USE_Boost)
    add_subdirectory(reservoirHandler)
    add_subdirectory(lrh)
endif()

#add_subdirectory(languageActionAnalysis)
