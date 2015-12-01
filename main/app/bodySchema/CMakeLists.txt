# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Martina Zambelli
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if(WYSIWYD_USE_OTL)
    add_subdirectory(modelOTL)
endif(WYSIWYD_USE_OTL)

add_subdirectory(babbling)
add_subdirectory(sensoryProcessor)

