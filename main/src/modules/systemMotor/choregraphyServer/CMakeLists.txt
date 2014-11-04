# Copyright: (C) 2011 EFAA Consortium
# Authors: Stephane Lallee
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME choregraphyServer)
project(${PROJECTNAME})

set(folder_source main.cpp)
source_group("Source Files" FILES ${folder_source})

include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${wrdac_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER systemMotor)
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} wrdac)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
