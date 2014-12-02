# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Hyung Jin Chang
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#
# demoModule CMake file, freely inspired by the iCub demoModule.
#

set(PROJECTNAME faceTracker)
project(${PROJECTNAME})

find_package(ICUB)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
    ${wrdac_INCLUDE_DIRS}
    ${ICUB_INCLUDE_DIRS} 	
)

# Search for source code.
file(GLOB sources src/*.cpp src/*.cc src/*.c)
file(GLOB headers include/*.h)

set(docs ${PROJECTNAME}.xml)

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})
source_group("DOC Files"    FILES ${docs})


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${headers} ${sources} ${docs})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} wrdac icubmod ${OpenCV_LIBRARIES}) 
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER attentionRelated)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${docs} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
