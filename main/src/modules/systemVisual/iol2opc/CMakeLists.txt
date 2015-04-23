# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Ugo Pattacini, Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME iol2opc)
project(${PROJECTNAME})

set(folder_header include/utils.h
                  include/classifierHandling.h
                  include/module.h)

set(folder_source src/utils.cpp
                  src/classifierHandling.cpp
                  src/module.cpp
                  src/main.cpp)

source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${wrdac_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ctrlLib ${YARP_LIBRARIES} wrdac)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER systemVisual)
install(TARGETS ${PROJECTNAME} DESTINATION bin)


