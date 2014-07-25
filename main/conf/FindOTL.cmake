# Copyright: 
# Author: 
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Created:
# OTL_INCLUDE_DIRS - Directories to include to use OTL
# OTL_LIBRARIES    - Default library to link against to use OTL
# OTL_FOUND        - If false, don't try to use OTL



SET (OTL_DIR $ENV{OTL_DIR})
message(${OTL_DIR})
SET (OTL_INCLUDE_DIRS $ENV{OTL_DIR}/src/libOTL)
message(${OTL_INCLUDE_DIRS})

FIND_PATH(OTL_INCLUDE_DIRS ${OTL_DIR}/src/libOTL)

MARK_AS_ADVANCED(OTL_INCLUDE_DIRS)
message(${OTL_INCLUDE_DIRS})

FIND_LIBRARY(OTL_LIBRARIES libOTL.a ${OTL_DIR}/build)
MARK_AS_ADVANCED(OTL_LIBRARIES)
message(${OTL_INCLUDE_DIRS})

if (OTL_INCLUDE_DIRS AND OTL_LIBRARIES)
	set(OTL_FOUND TRUE)
	message(OTL_FOUND)
endif(OTL_INCLUDE_DIRS AND OTL_LIBRARIES)


if (OTL_FOUND)
    set(OTL_INCLUDE_DIRS ${OTL_INCLUDE_DIRS})
else(OTL_FOUND)
    set(OTL_LIBRARIES "")
    set(OTL_INCLUDE_DIRS "")
endif(OTL_FOUND)


