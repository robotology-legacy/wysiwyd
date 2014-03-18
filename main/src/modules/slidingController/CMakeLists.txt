# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME slidingController)
project(${PROJECTNAME})

set(folder_source main.cpp)
source_group("Source Files" FILES ${folder_source})

include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} actionPrimitives iKin ctrlLib icubmod ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
