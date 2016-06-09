# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME learnPrimitive)
project(${PROJECTNAME})

find_package(LibLZMA)

file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/*.h  )
set(doc_files ${PROJECTNAME}.xml)


source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})
source_group("DOC Files"    FILES ${doc_files})

message("in learnPrimitive: R libraries: ${R_LIBRARIES}")
message("in learnPrimitive: R include dirs: ${R_INCLUDE_DIR} and Rcpp: ${Rcpp_INCLUDE_DIR} and RInside : ${RInside_INCLUDE_DIR}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wrdac_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${R_INCLUDE_DIR} ${Rcpp_INCLUDE_DIR} ${RInside_INCLUDE_DIR})

add_executable(${PROJECTNAME} ${folder_header} ${folder_source} ${doc_files})
target_link_libraries(${PROJECTNAME} wrdac ${YARP_LIBRARIES} ${LIBLZMA_LIBRARIES} ${R_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})

