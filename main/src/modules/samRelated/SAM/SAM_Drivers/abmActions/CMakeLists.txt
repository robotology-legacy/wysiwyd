# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Luke Boorman, Uriel Martinez
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME abmActions)
project(${PROJECTNAME})

find_package(YARP REQUIRED)

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

SET(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${PGSQL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${ACE_INCLUDE_DIR})
SET(LIBRARIES wrdac ${YARP_LIBRARIES} ${PGSQL_LIBRARIES} ${OpenCV_LIBRARIES} ${ACE_LIBRARIES})


#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
#target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES})
target_link_libraries(${PROJECTNAME} ${LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER samRelated)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${docs} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
