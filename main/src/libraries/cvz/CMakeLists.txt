# Copyright: (C) 2014 EFAA Consortium
# Authors: Stephane Lallee
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME cvz)
project(${PROJECTNAME})

include(YarpIDL)

file(GLOB folder_source src/*.cpp src/*.cc src/*.c 
	${IDL_GENERATED_CODE_DIR}/src/cvzMmcm_IDL.cpp
	)
file(GLOB folder_header include/cvz/*.h  
	${IDL_GENERATED_CODE_DIR}/include/cvzMmcm_IDL.h
	)
file(GLOB folder_header_core include/cvz/core/*.h)
file(GLOB folder_header_gui include/cvz/gui/*.h)
file(GLOB folder_header_helpers include/cvz/helpers/*.h)
set(doc_files cvz.xml cvzMmcm.xml)
set(idl_files_mmcm cvzMmcm.thrift)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})
source_group("Header Files: Core" FILES ${folder_header_core})
source_group("Header Files: Gui" FILES ${folder_header_gui})
source_group("Header Files: Helpers" FILES ${folder_header_helpers})
source_group("DOC Files"    FILES ${doc_files})
source_group("IDL Files"    FILES ${idl_files_mmcm})
yarp_idl_to_dir(${idl_files_mmcm} ${IDL_GENERATED_CODE_DIR})

find_package(GTK2)
add_definitions(${GTK2_DEFINITIONS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${IDL_GENERATED_CODE_DIR}/include ${YARP_INCLUDE_DIRS} ${GTK2_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${folder_source} ${folder_header} ${folder_header_core} ${folder_header_gui} ${folder_header_helpers} ${idl_files_cvz} ${idl_files_mmcm} ${doc_files})

set_target_properties(${PROJECTNAME} PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} 
									 ${OpenCV_LIBRARIES}
                                     ${GTK2_GTK_LIBRARY}
                                     ${GTK2_GLIB_LIBRARY}
                                     ${GTK2_GDK_LIBRARY}
                                     ${GTK2_GDK_PIXBUF_LIBRARY}
                                     ${GTK2_GOBJECT_LIBRARY}
                                     ${GTK2_GTHREAD_LIBRARY})
									 
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER ConVergenceZone)

icub_export_library(${PROJECTNAME} 
INTERNAL_INCLUDE_DIRS 
	${PROJECT_SOURCE_DIR}/include 
	${IDL_GENERATED_CODE_DIR}/include
DESTINATION 
	include/
FILES 
	${folder_header})

yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

