# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME slidingController)
project(${PROJECTNAME})

include(YarpIDL)

file(GLOB headers include/*.h)
file(GLOB sources src/*.cpp)
set(idl ${PROJECTNAME}.thrift)
set(doc ${PROJECTNAME}.xml)

yarp_idl_to_dir(${idl} ${PROJECT_SOURCE_DIR})

source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})
source_group("IDL Files"    FILES ${idl})
source_group("DOC Files"    FILES ${doc})

include_directories(${PROJECT_SOURCE_DIR}/include ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${headers} ${sources} ${idl} ${doc})
target_link_libraries(${PROJECTNAME} actionPrimitives iKin ctrlLib icubmod ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

