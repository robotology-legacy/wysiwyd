# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Luke Boorman, Uriel Martinez

set(appname visionDriver)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
file(GLOB template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

yarp_install(FILES ${conf} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${template} DESTINATION ${WYSIWYD_APPLICATIONS_TEMPLATES_INSTALL_DIR})

