# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME postureRecorder)
project(${PROJECTNAME})

set(source_files main.cpp)
set(doc_files ${PROJECTNAME}.xml)

source_group("Source Files" FILES ${source_files})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${YARP_INCLUDE_DIRS} ${wrdac_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${source_files} ${doc_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} wrdac)
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER systemMotor)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

