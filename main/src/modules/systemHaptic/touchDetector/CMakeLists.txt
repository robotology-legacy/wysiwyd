# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Bertrand Higy
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
project(touchDetector)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
  
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

set(folder_source main.cpp)
add_executable(${PROJECT_NAME} ${folder_source} touchDetectorModule.cpp touchDetectorThread.cpp)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER systemHaptic)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
