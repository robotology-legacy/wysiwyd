# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(AREiCubClientExample)
add_subdirectory(wrdacExample)
add_subdirectory(cvzStackExample)
if(YARP_VERSION_STRING VERSION_GREATER "2.3.63.1")
    add_subdirectory(ABMAugmentionExample)
endif()
