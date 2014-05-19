# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME opcInspector)
project(${PROJECTNAME})

set(header_files include/opcInspector.h)
set(source_files src/opcInspector.cpp src/main.cpp)
set(doc_files ${PROJECTNAME}.xml)

source_group("Header Files" FILES ${header_files})
source_group("Source Files" FILES ${source_files})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${YARP_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include
                    ${wrdac_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${header_files} ${source_files} ${doc_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} wrdac)
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER opcRelated)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
