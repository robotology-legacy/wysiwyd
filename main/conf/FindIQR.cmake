# Copyright: 2008-2010 RobotCub Consortium
# Author: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Created:
# IQR_INCLUDE_DIRS - Directories to include to use IQR
# IQR_LIBRARIES    - Default library to link against to use IQR
# IQR_FOUND        - If false, don't try to use IQR

IF (DEFINED ENV{IQR_ROOT})
	SET (IQR_DIR $ENV{IQR_ROOT})	
	message(STATUS "IQR Setup:")
	message(STATUS ${IQR_DIR})
	SET (IQR_INCLUDE_DIRS ${IQR_DIR})
	#FIND_PATH(IQR_INCLUDE_DIRS Common/Item/neuron.hpp ${IQR_DIR})
	message(STATUS ${IQR_INCLUDE_DIRS})
	MARK_AS_ADVANCED(IQR_DIR)
	MARK_AS_ADVANCED(IQR_INCLUDE_DIRS)
ENDIF (DEFINED ENV{IQR_ROOT})

SET (IQR_FOUND FALSE)
if (IQR_INCLUDE_DIRS)
	set(IQR_FOUND TRUE)
endif(IQR_INCLUDE_DIRS)

if (NOT IQR_FOUND)
    set(IQR_LIBRARIES "")
    set(IQR_INCLUDE_DIRS "")
endif(NOT IQR_FOUND)

