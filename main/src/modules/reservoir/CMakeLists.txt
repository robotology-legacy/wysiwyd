# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Anne-Laure MEALIER
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(bodyReservoir)
add_subdirectory(lrh)

if(WYSIWYD_USE_Boost)
    add_subdirectory(reservoirHandler)
endif()
#add_subdirectory(languageActionAnalysis)
