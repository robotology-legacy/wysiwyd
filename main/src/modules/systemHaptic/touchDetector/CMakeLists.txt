# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Bertrand Higy
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME touchDetector)
project(${PROJECTNAME})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
  
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

set(folder_source main.cpp)
#source_group("Source Files" FILES ${folder_source})
add_executable(${PROJECTNAME} ${folder_source} touchDetectorModule.cpp touchDetectorThread.cpp)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)
