# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Grégoire Pointeau
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME keyboardBabbling)
project(${PROJECTNAME})


file(GLOB folder_source src/main.cpp)
set(doc_files ${PROJECTNAME}.xml)


source_group("Source Files" FILES ${folder_source})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} )

add_executable(${PROJECTNAME} ${folder_source} ${doc_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} iKin)
set_target_properties(${PROJECTNAME} PROPERTIES FOLDER systemMotor)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

