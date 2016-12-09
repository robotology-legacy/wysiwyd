# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Grégoire Pointeau, Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME autobiographicalMemory)
project(${PROJECTNAME})

file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h include/db/*.h)
set(docs ${PROJECTNAME}.xml)

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})
source_group("DOC Files"    FILES ${docs})

find_package( Threads )

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${PGSQL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${ACE_INCLUDE_DIR}
)

SET(LIBRARIES
    ${ACE_LIBRARIES} ${YARP_LIBRARIES} ${PGSQL_LIBRARIES} ${OpenCV_LIBRARIES} wrdac ${CMAKE_THREAD_LIBS_INIT}
)

include_directories(${INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${headers} ${sources} ${docs})
target_link_libraries(${PROJECTNAME} ${LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER AutoBiographicalMemory)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

