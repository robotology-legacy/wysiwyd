# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Grégoire Pointeau
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname ears)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)

yarp_install(FILES ${conf} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})

