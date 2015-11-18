# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Anne-Laure Mealier
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME reservoirHandler)
project(${PROJECTNAME})


file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/*.h  )
set(doc_files ${PROJECTNAME}.xml)


source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})
source_group("DOC Files"    FILES ${doc_files})

FIND_PACKAGE(PythonLibs 2.7 REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#cmake -DPYTHON_INCLUDE=/usr/lib/python2.7 ..

add_executable(${PROJECTNAME} ${folder_header} ${folder_source} ${doc_files})
target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES} ${PYTHON_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

