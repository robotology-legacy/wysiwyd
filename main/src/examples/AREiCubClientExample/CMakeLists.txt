# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(AREiCubClientExample)

set(source_files main.cpp)
set(doc_files ${PROJECT_NAME}.xml)
source_group("Source Files" FILES ${source_files})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${source_files} ${doc_files})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Examples)
target_link_libraries(${PROJECT_NAME} wrdac ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

