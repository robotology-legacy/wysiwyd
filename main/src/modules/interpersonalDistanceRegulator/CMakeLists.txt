# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME interpersonalDistanceRegulator)
project(${PROJECTNAME})

include(YarpIDL)

set(headers ${IDL_GENERATED_CODE_DIR}/include/${PROJECTNAME}_IDL.h interpersonalDistanceRegulator.h)
set(sources ${IDL_GENERATED_CODE_DIR}/src/${PROJECTNAME}_IDL.cpp interpersonalDistanceRegulator.cpp main.cpp)

set(idl_files ${PROJECTNAME}.thrift)
set(docs ${PROJECTNAME}.xml)

yarp_idl_to_dir(${idl_files} ${IDL_GENERATED_CODE_DIR})

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})
source_group("IDL Files"    FILES ${idl_files})
source_group("DOC Files"    FILES ${docs})

include_directories(${IDL_GENERATED_CODE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${headers} ${sources} ${idl_files} ${docs})
target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
