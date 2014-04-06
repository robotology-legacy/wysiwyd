# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stephane Lallee
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME cvzHierarchyXmlBuilder)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)
source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${cvz_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} cvz)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ConVergenceZone)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
#yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

