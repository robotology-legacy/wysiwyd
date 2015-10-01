# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Grégoire Pointeau
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#add_subdirectory(frontalEyeField)


if (WYSIWYD_USE_OpenCV)
	add_subdirectory(brightnessSensor)
	add_subdirectory(iol2opc)
endif()


