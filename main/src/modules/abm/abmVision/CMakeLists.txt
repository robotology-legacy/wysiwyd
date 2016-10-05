# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME abmVision)
project(${PROJECTNAME})

file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h)
set(docs ${PROJECTNAME}.xml)

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})
source_group("DOC Files"    FILES ${docs})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${iKin_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${headers} ${sources} ${docs})
target_link_libraries(${PROJECTNAME} wrdac iKin ${YARP_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER AutoBiographicalMemory)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
