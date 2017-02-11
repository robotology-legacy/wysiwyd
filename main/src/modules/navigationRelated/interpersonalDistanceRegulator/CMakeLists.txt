# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME interpersonalDistanceRegulator)
project(${PROJECTNAME})

include(YarpIDL)

set(headers_files ${IDL_GENERATED_CODE_DIR}/include/${PROJECTNAME}_IDL.h ${PROJECTNAME}.h)
set(sources_files ${IDL_GENERATED_CODE_DIR}/src/${PROJECTNAME}_IDL.cpp ${PROJECTNAME}.cpp main.cpp)

set(idl_files ${PROJECTNAME}.thrift)
set(docs_files ${PROJECTNAME}.xml)

yarp_idl_to_dir(${idl_files} ${IDL_GENERATED_CODE_DIR})

source_group("Source Files" FILES ${sources_files})
source_group("Header Files" FILES ${headers_files})
source_group("IDL Files"    FILES ${idl_files})
source_group("DOC Files"    FILES ${docs_files})

include_directories(${IDL_GENERATED_CODE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${headers_files} ${sources_files} ${idl_files} ${docs_files})
target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER navigationRelated)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
