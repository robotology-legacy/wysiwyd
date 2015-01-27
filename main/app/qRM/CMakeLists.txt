# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname qRM)
file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
yarp_install(FILES ${ini} ${grxml} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})
