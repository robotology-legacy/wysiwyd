# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname headPoseEstimator)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.* ${CMAKE_CURRENT_SOURCE_DIR}/conf/trees/*.*)
file(GLOB template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

yarp_install(FILES ${conf} DESTINATION ${WYSIWYD_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${template} DESTINATION ${WYSIWYD_APPLICATIONS_TEMPLATES_INSTALL_DIR})

