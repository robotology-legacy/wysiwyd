# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(test-are-attention)
include_directories(${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} main.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Tests)
target_link_libraries(${PROJECT_NAME} wrdac ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

