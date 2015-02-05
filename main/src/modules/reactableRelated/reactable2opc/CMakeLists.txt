# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stephane Lallee
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME reactable2opc)
project(${PROJECTNAME})

add_definitions(-DOSC_HOST_LITTLE_ENDIAN)

file(GLOB folder_source *.cpp *.cc *.c)
file(GLOB folder_header *.h *.hpp)
set(doc_files ${PROJECTNAME}.xml)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})
source_group("TUIO System Specific Files" FILES ${OSC_SYSTEM_SPECIFIC_SRC})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${YARP_INCLUDE_DIRS} ${TUIO_INCLUDE_DIRS} ${wrdac_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header} ${OSC_SYSTEM_SPECIFIC_SRC} ${doc_files})
#add_executable(${PROJECTNAME} ${folder_source} ${folder_header} )
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER reactableRelated)
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${TUIO_LIBRARIES} wrdac pthread)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

