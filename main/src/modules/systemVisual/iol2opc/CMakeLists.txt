# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Ugo Pattacini, Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME iol2opc)
project(${PROJECTNAME})

include(YarpIDL)

set(folder_header include/utils.h
                  include/module.h
                  ${IDL_GENERATED_CODE_DIR}/include/${PROJECTNAME}_IDL.h)

set(folder_source src/utils.cpp
                  src/module.cpp
                  src/main.cpp
                  ${IDL_GENERATED_CODE_DIR}/src/${PROJECTNAME}_IDL.cpp)

set(idl_files ${PROJECTNAME}.thrift)

yarp_idl_to_dir(${idl_files} ${IDL_GENERATED_CODE_DIR})

source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})
source_group("IDL Files"    FILES ${idl_files})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${IDL_GENERATED_CODE_DIR}/include
                    ${wrdac_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${idl_files} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ctrlLib ${YARP_LIBRARIES} wrdac)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER systemVisual)
install(TARGETS ${PROJECTNAME} DESTINATION bin)


