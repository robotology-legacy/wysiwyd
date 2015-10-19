# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Matej Hoffmann
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if(ICUB_USE_IPOPT)
    add_subdirectory(referenceFrameHandler)
endif()

add_subdirectory(guiUpdater)
add_subdirectory(qRM)
add_subdirectory(abmInteraction)
add_subdirectory(InsermInteractionEngine)
add_subdirectory(opcPopulater)
if(WYSIWYD_USE_Boost)
    add_subdirectory(humanRobotDumper)
endif()
