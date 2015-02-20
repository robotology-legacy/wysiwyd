# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME proprioRosYarpExample)
project(${PROJECTNAME})

set(docs ${PROJECTNAME}.xml)
set(sources main.cpp proprioRosYarpExample.cpp)
set(headers proprioRosYarpExample.h)
set(doc_files ${PROJECTNAME}.xml)

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})
source_group("DOC Files"    FILES ${docs})

include_directories(${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${headers} ${sources} ${docs})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Examples)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

