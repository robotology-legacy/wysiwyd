# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(wrdacExample)

set(folder_source main.cpp)
set(doc_files ${PROJECT_NAME}.xml)
source_group("Source Files" FILES ${folder_source})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${doc_files})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Examples)
target_link_libraries(${PROJECT_NAME} wrdac ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

