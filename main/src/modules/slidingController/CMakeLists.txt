# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME slidingController)
project(${PROJECTNAME})

include(YarpIDL)

set(header_files ${IDL_GENERATED_CODE_DIR}/include/${PROJECTNAME}_IDL.h)
set(source_files ${IDL_GENERATED_CODE_DIR}/src/${PROJECTNAME}_IDL.cpp main.cpp)
set(idl_files ${PROJECTNAME}.thrift)
set(doc_files ${PROJECTNAME}.xml)

yarp_idl_to_dir(${idl_files} ${IDL_GENERATED_CODE_DIR})

source_group("Header Files" FILES ${header_files})
source_group("Source Files" FILES ${source_files})
source_group("IDL Files"    FILES ${idl_files})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${IDL_GENERATED_CODE_DIR}/include ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${header_files} ${source_files} ${idl_files} ${doc_files})
target_link_libraries(${PROJECTNAME} actionPrimitives iKin ctrlLib icubmod ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

