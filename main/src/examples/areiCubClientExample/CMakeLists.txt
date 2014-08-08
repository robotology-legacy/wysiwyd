# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME areiCubClientExample)
project(${PROJECTNAME})

set(source_files main.cpp)
set(doc_files ${PROJECTNAME}.xml)
source_group("Source Files" FILES ${source_files})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${source_files} ${doc_files})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Examples)
target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

