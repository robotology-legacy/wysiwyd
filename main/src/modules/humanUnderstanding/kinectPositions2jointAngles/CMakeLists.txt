# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME kinectPositions2jointAngles)
project(${PROJECTNAME})

file(GLOB folder_source *.h *.cpp)
source_group("Source Files" FILES ${folder_source})

include_directories(${kinectWrapper_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} kinectWrapper ${YARP_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER humanUnderstanding)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${docs} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

