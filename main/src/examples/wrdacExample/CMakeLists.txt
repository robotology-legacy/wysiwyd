# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME wrdacExample)
project(${PROJECTNAME})

file(GLOB folder_source main.cpp)
set(doc_files ${PROJECTNAME}.xml)
source_group("Source Files" FILES ${folder_source})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header} ${doc_files})
target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

