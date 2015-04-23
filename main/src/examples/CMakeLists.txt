# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(AREiCubClientExample)
add_subdirectory(wrdacExample)
add_subdirectory(ABMRosToYarpBridge)

if(YARP_VERSION_STRING VERSION_GREATER "2.3.63.1") #because of copyPortable
    add_subdirectory(ABMAugmentionExample)
endif()

if(WYSIWYD_USE_GTK2)
    add_subdirectory(cvzStackExample)
endif()
