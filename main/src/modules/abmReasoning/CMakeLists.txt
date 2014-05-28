# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Grégoire Pointeau
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME abmReasoning)
project(${PROJECTNAME})

set(header_files include/*.h )
set(source_files src/*.cpp )
set(doc_files ${PROJECTNAME}.xml)

source_group("Header Files" FILES ${header_files})
source_group("Source Files" FILES ${source_files})

source_group("DOC Files"    FILES ${doc_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${header_files} ${source_files} ${doc_files})
target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})