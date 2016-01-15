# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Ugo Pattacini, Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(iol2opc)
set(WYSIWYD_IOL2OPC_WITH_TRACKING FALSE CACHE BOOL "Enable iol2opc with tracking (requires VERSION(OpenCV) > 2.x.y)")

include(YarpIDL)

set(folder_header include/utils.h
                  include/module.h
                  ${IDL_GENERATED_CODE_DIR}/include/${PROJECT_NAME}_IDL.h)

set(folder_source src/utils.cpp
                  src/module.cpp
                  src/main.cpp
                  ${IDL_GENERATED_CODE_DIR}/src/${PROJECT_NAME}_IDL.cpp)

set(idl_files ${PROJECT_NAME}.thrift)

yarp_idl_to_dir(${idl_files} ${IDL_GENERATED_CODE_DIR})

source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})
source_group("IDL Files"    FILES ${idl_files})

if(WYSIWYD_IOL2OPC_WITH_TRACKING)
    add_definitions(-DTRACKING)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${IDL_GENERATED_CODE_DIR}/include
                    ${wrdac_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${idl_files} ${folder_header} ${folder_source})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ctrlLib ${YARP_LIBRARIES} wrdac)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER systemVisual)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
