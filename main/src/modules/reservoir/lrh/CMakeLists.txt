# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Anne-Laure Mealier
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(PythonLibs 2.7)
if(PythonLibs_FOUND)
  project(lrh)

  file(GLOB sources src/*.cpp)
  file(GLOB headers include/*.h)
  set(docs ${PROJECT_NAME}.xml)

  source_group("Source Files" FILES ${sources})
  source_group("Header Files" FILES ${headers})
  source_group("DOC Files"    FILES ${docs})

  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                      ${wrdac_INCLUDE_DIRS}
                      ${YARP_INCLUDE_DIRS}
                      ${PYTHON_INCLUDE_DIRS})

  add_executable(${PROJECT_NAME} ${headers} ${sources} ${docs})
  target_link_libraries(${PROJECT_NAME} wrdac ${YARP_LIBRARIES} ${PYTHON_LIBRARIES})
  set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER lrh)
  install(TARGETS ${PROJECT_NAME} DESTINATION bin)
  yarp_install(FILES ${doc_files} DESTINATION ${WYSIWYD_MODULES_INSTALL_DIR})
else()
  message(STATUS "Python 2.7 is not found ==> lrh is skipped")
endif()

