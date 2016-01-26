# Copyright: (C) 2016 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if(ICUB_USE_IPOPT)
   add_subdirectory(referenceFrameHandler)
endif()

add_subdirectory(qRM)
add_subdirectory(abmInteraction)
add_subdirectory(opcPopulater)
add_subdirectory(humanRobotDumper)
add_subdirectory(actionRecogDataDumper)

