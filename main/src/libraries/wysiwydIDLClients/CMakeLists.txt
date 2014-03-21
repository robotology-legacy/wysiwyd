# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME wysiwydIDLClients)
project(${PROJECTNAME})

file(GLOB header_files ${IDL_GENERATED_CODE_DIR}/include/*.h)
file(GLOB source_files ${IDL_GENERATED_CODE_DIR}/src/*.cpp)

list(APPEND header_files ${PROJECTNAME}.h)

source_group("Header Files" FILES ${header_files})
source_group("Source Files" FILES ${source_files})

include_directories(${IDL_GENERATED_CODE_DIR}/include ${YARP_INCLUDE_DIRS})
add_library(${PROJECTNAME} ${header_files} ${source_files})
set_target_properties(${PROJECTNAME} PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${IDL_GENERATED_CODE_DIR}/include
                                   DESTINATION include/wysiwyd/${PROJECTNAME}
                                   FILES ${header_files})
