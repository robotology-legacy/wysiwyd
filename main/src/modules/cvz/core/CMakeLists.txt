# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stephane Lallee
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME cvzCore)
project(${PROJECTNAME})
#set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ConVergenceZone)
find_package(GTK2)
add_definitions(${GTK2_DEFINITIONS})
include(YarpIDL)

file(GLOB folder_source *.cpp ${IDL_GENERATED_CODE_DIR}/src/${PROJECTNAME}_IDL.cpp)
file(GLOB folder_header *.h ${IDL_GENERATED_CODE_DIR}/include/${PROJECTNAME}_IDL.h)
set(doc_files ${PROJECTNAME}.xml)
set(idl_files ${PROJECTNAME}.thrift)
source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})

source_group("DOC Files"    FILES ${doc_files})
source_group("IDL Files"    FILES ${idl_files})
yarp_idl_to_dir(${idl_files} ${IDL_GENERATED_CODE_DIR})

include_directories(${IDL_GENERATED_CODE_DIR}/include ${YARP_INCLUDE_DIRS} ${cvz_INCLUDE_DIRS} ${GTK2_INCLUDE_DIRS})
#include_directories(${YARP_INCLUDE_DIRS} ${cvz_INCLUDE_DIRS} ${GTK2_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header} ${idl_files} ${doc_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} 
									 cvz
									 ${GTK2_GTK_LIBRARY}
                                     ${GTK2_GLIB_LIBRARY}
                                     ${GTK2_GDK_LIBRARY}
                                     ${GTK2_GDK_PIXBUF_LIBRARY}
                                     ${GTK2_GOBJECT_LIBRARY}
                                     ${GTK2_GTHREAD_LIBRARY})
									 
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ConVergenceZone)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

