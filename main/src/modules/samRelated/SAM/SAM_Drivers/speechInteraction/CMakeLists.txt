# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Uriel Martinez, Luke Boorman
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME speechInteraction)
project(${PROJECTNAME})

find_package(YARP REQUIRED)

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

IF(WYSIWYD_USE_Boost) #Boost is an optional dependency!
    SET(LIBRARIES ${LIBRARIES} ${Boost_LIBRARIES})
    SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
ENDIF()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES} ${LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER samRelated)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${docs} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
