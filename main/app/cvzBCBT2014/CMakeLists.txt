# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname cvzBCBT2014)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
yarp_install(FILES ${conf} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})
file(GLOB template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.*)
yarp_install(FILES ${template} DESTINATION ${WYSIWYD_APPLICATIONS_TEMPLATES_INSTALL_DIR})

