# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Grégoire Pointeau
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME opcManager)
project(${PROJECTNAME})

include(YarpIDL)

set(source_files ${IDL_GENERATED_CODE_DIR}/src/${PROJECTNAME}_IDL.cpp src/main.cpp src/opcManager.cpp)
set(header_files include/*.h ${IDL_GENERATED_CODE_DIR}/include/${PROJECTNAME}_IDL.h)
set(idl_files ${PROJECTNAME}.thrift)
set(doc_files ${PROJECTNAME}.xml)

yarp_idl_to_dir(${idl_files} ${IDL_GENERATED_CODE_DIR})

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})
source_group("IDL Files"    FILES ${idl_files})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${header_files} ${source_files} ${idl_files} ${doc_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

