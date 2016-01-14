# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#add_subdirectory(frontalEyeField)

if (WYSIWYD_USE_GTK2)
    add_subdirectory(cvzVisualHierarchy)
endif()

if (WYSIWYD_USE_OpenCV)
    add_subdirectory(imageSplitter)
    add_subdirectory(imageFuser)
    add_subdirectory(caltechDatasetPlayer)
    add_subdirectory(brightnessSensor)
    add_subdirectory(iol2opc)
endif()


