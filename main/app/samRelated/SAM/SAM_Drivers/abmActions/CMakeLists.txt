# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Luke Boorman, Uriel Martinez

set(appname abmActions)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
yarp_install(FILES ${conf} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})
