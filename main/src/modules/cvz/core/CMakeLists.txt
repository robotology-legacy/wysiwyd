# Copyright: (C) 2011 EFAA Consortium
# Authors: Maxime Petit, Stephane Lallee
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME cvzCore)
project(${PROJECTNAME})

find_package(GTK2)
add_definitions(${GTK2_DEFINITIONS})

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)
source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})
message("GTK2 = " ${GTK2_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${GTK2_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} 
								   ${GTK2_GTK_LIBRARY}
                                   ${GTK2_GLIB_LIBRARY}
                                   ${GTK2_GDK_LIBRARY}
                                   ${GTK2_GDK_PIXBUF_LIBRARY}
                                   ${GTK2_GOBJECT_LIBRARY}
                                   ${GTK2_GTHREAD_LIBRARY})

install(TARGETS ${PROJECTNAME} DESTINATION bin)
#yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

