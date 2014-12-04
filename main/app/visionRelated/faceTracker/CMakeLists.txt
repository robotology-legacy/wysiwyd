# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Grégoire Pointeau the terms of the GNU GPL v2.0.

set(appname faceTracker)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
yarp_install(FILES ${conf} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})
