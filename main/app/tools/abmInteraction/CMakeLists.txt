# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname abmInteraction)
file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
yarp_install(FILES ${ini} ${grxml} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})

