# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Clément Moulin-Frier, Jordi Puigbo, Low Sock Ching 
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname planner)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
yarp_install(FILES ${conf} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})

