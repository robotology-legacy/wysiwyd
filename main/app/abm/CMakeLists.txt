# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Grégoire Pointeau
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if(WYSIWYD_USE_PGSQL)
    add_subdirectory(autobiographicalMemory)
endif(WYSIWYD_USE_PGSQL)

add_subdirectory(ABMRosToYarpBridge)
add_subdirectory(abmHandler)
add_subdirectory(abmReasoning)

