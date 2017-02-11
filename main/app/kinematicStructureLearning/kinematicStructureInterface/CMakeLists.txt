# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname kinematicStructureInterface)
file(GLOB ini      ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
yarp_install(FILES ${ini}      DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})

