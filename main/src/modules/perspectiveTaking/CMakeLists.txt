# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Caution when trying to use VTK6.0 with QT5!!!
# VTK6.0 must be compiled with support for QT5, see http://www.vtk.org/Wiki/VTK/Configure_and_Build

include(wysiwydUseQt5)

set(PROJECTNAME perspectiveTaking)
project(${PROJECTNAME})

include(YarpIDL)

file(GLOB sources src/*.cpp ${IDL_GENERATED_CODE_DIR}/src/${PROJECTNAME}_IDL.cpp)
file(GLOB headers include/*.h ${IDL_GENERATED_CODE_DIR}/include/${PROJECTNAME}_IDL.h)
set(docs ${PROJECTNAME}.xml)

set(idl_files ${PROJECTNAME}.thrift)

yarp_idl_to_dir(${idl_files} ${IDL_GENERATED_CODE_DIR})

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})
source_group("DOC Files"    FILES ${docs})
source_group("IDL Files"    FILES ${idl_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${IDL_GENERATED_CODE_DIR}/include
                    ${wrdac_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${kinectWrapper_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${RTABMap_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${headers} ${idl_files} ${sources} ${docs})
target_link_libraries(  ${PROJECTNAME}
                        wrdac
                        ${YARP_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        ${kinectWrapper_LIBRARIES}
                        ${PCL_COMMON_LIBRARIES}
                        ${PCL_IO_LIBRARIES}
                        ${PCL_FILTERS_LIBRARIES}
                        ${PCL_VISUALIZATION_LIBRARIES}
                        rtabmap_core rtabmap_utilite)
qt5_use_modules(${PROJECTNAME} Widgets)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER perspectiveTaking)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
