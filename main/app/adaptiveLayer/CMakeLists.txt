# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname adaptiveLayer)
file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB grxml ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.grxml)
yarp_install(FILES ${ini} ${grxml} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})

