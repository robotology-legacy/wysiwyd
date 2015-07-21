# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Matej Hoffmann
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(cartControlReachAvoid)

file(GLOB source *.cpp)
file(GLOB header *.h)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(cartControlReachAvoid ${header} ${source})
target_link_libraries(cartControlReachAvoid iKin ctrlLib ${YARP_LIBRARIES})
set_target_properties(cartControlReachAvoid PROPERTIES FOLDER cartControlReachAvoid)
install(TARGETS cartControlReachAvoid DESTINATION bin)

