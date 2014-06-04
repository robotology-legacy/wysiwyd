# dopyright: (C) 2014 WYSIWYD Consortium
# Authors: Hector Barron-Gonzalez
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME awareTouch)
project(${PROJECTNAME})

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/*.h)
set(doc_files ${PROJECTNAME}.xml)
source_group("Source Files" FILES ${source_files})
source_group("Header Files" FILES ${header_files})
source_group("DOC Files" FILES ${doc_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${ICUB_INCLUDE_DIRS}
                    ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${skinDynLib_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${source_files} ${header_files} ${doc_files})
target_link_libraries(${PROJECTNAME} iKin iDyn actionPrimitives perceptiveModels skinDynLib icubmod wrdac ${YARP_LIBRARIES}) 
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
