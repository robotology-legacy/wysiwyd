# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Grégoire Pointeau, Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname abmReasoning)

file(GLOB conf     ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
file(GLOB template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

yarp_install(FILES ${conf}     DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${template} DESTINATION ${WYSIWYD_APPLICATIONS_TEMPLATES_INSTALL_DIR})