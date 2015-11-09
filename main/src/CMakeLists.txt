# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(libraries)
add_subdirectory(examples)
add_subdirectory(modules)

option(COMPILE_TESTS "Compile Tests" OFF)
if(COMPILE_TESTS)
    add_subdirectory(tests)
endif()

