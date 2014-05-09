# Copyright: (C) 2011 EFAA Consortium
# Authors: Lorenzo Natale, Giorgio Metta
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#
# demoModule CMake file, freely inspired by the iCub demoModule.
#

cmake_minimum_required(VERSION 2.6)


set(PROJECTNAME "frontalEyeField")
project(${PROJECTNAME})

find_package(ICUB)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
    ${efaaHelpers_INCLUDE_DIRS}
    ${ICUB_INCLUDE_DIRS} 	
)



# Search for source code.
file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(${PROJECTNAME} 
	${folder_source} 
	${folder_header}
)
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER visualSystem)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} icubmod) 

install(TARGETS ${PROJECTNAME} DESTINATION bin)

