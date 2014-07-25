# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Anne-Laure MEALIER
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname bodySchema)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
yarp_install(FILES ${conf} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})
