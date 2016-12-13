# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Ugo Pattacini, Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(WYSIWYD_BUILD_IOL2OPC TRUE CACHE BOOL "Enable compilation of iol2opc")
mark_as_advanced(WYSIWYD_BUILD_IOL2OPC)
if(WYSIWYD_BUILD_IOL2OPC)
   project(iol2opc)

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

   include_directories(${PROJECT_SOURCE_DIR}/include
                       ${IDL_GENERATED_CODE_DIR}/include
                       ${wrdac_INCLUDE_DIRS}
                       ${OpenCV_INCLUDE_DIRS}
                       ${YARP_INCLUDE_DIRS}
                       ${ICUB_INCLUDE_DIRS} )

   add_executable(${PROJECT_NAME} ${idl_files} ${folder_header} ${folder_source})
   target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ctrlLib ${YARP_LIBRARIES} wrdac)

   set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER systemVisual)
   install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif()

add_custom_target(reachable_area_build ALL)
add_custom_command(TARGET reachable_area_build POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/reachableArea.py ${CMAKE_BINARY_DIR}/bin
                   COMMENT "Copying reachableArea.py to ${CMAKE_BINARY_DIR}/bin")

install(PROGRAMS "./reachableArea.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)
yarp_install(PROGRAMS "./reachableArea.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)

