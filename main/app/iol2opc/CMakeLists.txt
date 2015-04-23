# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Ugo Patticini, Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname iol2opc)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
file(GLOB template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

yarp_install(FILES ${conf} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${template} DESTINATION ${WYSIWYD_APPLICATIONS_TEMPLATES_INSTALL_DIR})

