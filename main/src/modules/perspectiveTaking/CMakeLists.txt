# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME perspectiveTaking)
project(${PROJECTNAME})

file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h)
set(docs ${PROJECTNAME}.xml)

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})
source_group("DOC Files"    FILES ${docs})

include(${QT_USE_FILE})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${kinectWrapper_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${RTABMap_INCLUDE_DIRS} ${QVTK_INCLUDE_DIR})

qt4_wrap_cpp(moc_srcs ${headers})

add_executable(${PROJECTNAME} ${headers} ${sources} ${docs} ${moc_srcs})
target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${kinectWrapper_LIBRARIES} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${RTABMap_LIBRARIES} QVTK)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER perspectiveTaking)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
