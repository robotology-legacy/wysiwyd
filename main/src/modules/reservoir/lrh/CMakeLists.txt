# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Anne-Laure Mealier
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME lrh)
project(${PROJECTNAME})

file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h)
set(docs ${PROJECTNAME}.xml)

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})
source_group("DOC Files"    FILES ${docs})

find_package(PythonLibs 2.7 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${wrdac_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${PYTHON_INCLUDE_DIRS}
					${Boost_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${headers} ${sources} ${docs})
target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${PYTHON_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER lrh)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

