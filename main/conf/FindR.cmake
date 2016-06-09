
#
# - This module locates an installed R distribution.
#
# Input:
#  R_LIB_ARCH - For windows (i386 or x64)
#
# Defines the following:
#  R_COMMAND           - Path to R command
#  R_HOME              - Path to 'R home', as reported by R
#  R_INCLUDE_DIR       - Path to R include directory
#  R_LIBRARY_BASE      - Path to R library
#  R_LIBRARY_BLAS      - Path to Rblas / blas library
#  R_LIBRARY_LAPACK    - Path to Rlapack / lapack library
#  R_LIBRARY_READLINE  - Path to readline library
#  R_LIBRARIES         - Array of: R_LIBRARY_BASE, R_LIBRARY_BLAS, R_LIBRARY_LAPACK, R_LIBRARY_BASE [, R_LIBRARY_READLINE]
#
#  VTK_R_HOME          - (deprecated, use R_HOME instead) Path to 'R home', as reported by R
#
# Variable search order:
#   1. Attempt to locate and set R_COMMAND
#     - If unsuccessful, generate error and prompt user to manually set R_COMMAND
#   2. Use R_COMMAND to set R_HOME
#   3. Locate other libraries in the priority:
#     1. Within a user-built instance of R at R_HOME
#     2. Within an installed instance of R
#     3. Within external system libraries
#

set(TEMP_CMAKE_FIND_APPBUNDLE ${CMAKE_FIND_APPBUNDLE})
set(CMAKE_FIND_APPBUNDLE "NEVER")
find_program(R_COMMAND R DOC "R executable.")
set(CMAKE_FIND_APPBUNDLE ${TEMP_CMAKE_FIND_APPBUNDLE})

if(NOT R_LIB_ARCH)
  set(R_LIB_ARCH "ThisPathNotExists")
endif()

if(R_COMMAND)
  execute_process(WORKING_DIRECTORY .
                  COMMAND ${R_COMMAND} RHOME
                  OUTPUT_VARIABLE R_ROOT_DIR
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  # deprecated
  if(VTK_R_HOME)
    set(R_HOME ${VTK_R_HOME} CACHE PATH "R home directory obtained from R RHOME")
  else(VTK_R_HOME)
    set(R_HOME ${R_ROOT_DIR} CACHE PATH "R home directory obtained from R RHOME")
    set(VTK_R_HOME ${R_HOME})
  endif(VTK_R_HOME)
  # /deprecated
  # the following command does nothing currently, but will be used when deprecated code is removed
  set(R_HOME ${R_ROOT_DIR} CACHE PATH "R home directory obtained from R RHOME")

  find_path(R_INCLUDE_DIR R.h
            HINTS ${R_ROOT_DIR}
            PATHS /usr/local/lib /usr/local/lib64 /usr/share
            PATH_SUFFIXES include R/include
            DOC "Path to file R.h")
  if(NOT R_INCLUDE_DIR)
    message(
        FATAL_ERROR
        "R.h file not found. Locations tried:\n"
        "/usr/local/lib /usr/local/lib64 /usr/share ${R_ROOT_DIR}"
    )
  endif()

  #RCPP
  file(GLOB_RECURSE RCPP_HINT "${R_HOME}/*/Rcpp.h")
  string(REGEX REPLACE "/Rcpp.h$" "" RCPP_HINT "${RCPP_HINT}")

  find_path(Rcpp_INCLUDE_DIR "Rcpp.h" HINTS "${RCPP_HINT}")

  #RInside
  file(GLOB_RECURSE RINSIDE_HINT "${R_HOME}/*/RInside.h")
  string(REGEX REPLACE "/RInside.h$" "" RINSIDE_HINT "${RINSIDE_HINT}")

  find_path(RInside_INCLUDE_DIR "RInside.h" HINTS "${RINSIDE_HINT}")

  find_library(R_LIBRARY_BASE R
            HINTS ${R_ROOT_DIR}/lib ${R_ROOT_DIR}/bin/${R_LIB_ARCH}
            DOC "R library (example libR.a, libR.dylib, etc.).")
  if(NOT R_LIBRARY_BASE)
    set(R_LIBRARY_BASE "")
    message(
        "R library not found. Locations tried:\n"
        "${R_ROOT_DIR}/lib ${R_ROOT_DIR}/bin/${R_LIB_ARCH}"
    )
    if(APPLE)
      message(FATAL_ERROR "")
    endif()
  endif()

  find_library(R_LIBRARY_BLAS NAMES Rblas blas
            HINTS ${R_ROOT_DIR}/lib ${R_ROOT_DIR}/bin/${R_LIB_ARCH}
            DOC "Rblas library (example libRblas.a, libRblas.dylib, etc.).")

  find_library(R_LIBRARY_LAPACK NAMES Rlapack lapack
            HINTS ${R_ROOT_DIR}/lib ${R_ROOT_DIR}/bin/${R_LIB_ARCH}
            DOC "Rlapack library (example libRlapack.a, libRlapack.dylib, etc.).")

  find_library(R_LIBRARY_RINSIDE NAMES Rrinside RInside
            HINTS ${R_ROOT_DIR}/lib ${R_ROOT_DIR}/bin/${R_LIB_ARCH} ${R_HOME}/site-library/RInside/lib
            DOC "Rinside library (example libRinside.a, libRinside.dylib, etc.).")

  find_library(R_LIBRARY_READLINE readline
            DOC "(Optional) system readline library. Only required if the R libraries were built with readline support.")

else(R_COMMAND)
  message(SEND_ERROR "FindR.cmake requires the following variables to be set: R_COMMAND")
endif(R_COMMAND)

# Note: R_LIBRARY_BASE is added to R_LIBRARIES twice; this may be due to circular linking dependencies; needs further investigation
set(R_LIBRARIES ${R_LIBRARY_BASE} ${R_LIBRARY_BLAS} ${R_LIBRARY_LAPACK} ${R_LIBRARY_BASE} ${R_LIBRARY_RINSIDE})
#if(R_LIBRARY_READLINE)
#  set(R_LIBRARIES ${R_LIBRARIES} ${R_LIBRARY_READLINE})
#endif(R_LIBRARY_READLINE)

if (R_COMMAND AND R_LIBRARIES)
	set(R_FOUND TRUE)
	message(STATUS "R Found!")
endif(R_COMMAND AND R_LIBRARIES)

if (R_FOUND)
    message("Test R is found!")
    message(${R_LIBRARIES})
    set(R_INCLUDE_DIRS ${Rcpp_INCLUDE_DIRS} ${RInside_INCLUDE_DIRS})
else(R_FOUND)
    set(R_LIBRARIES "")
    set(R_INCLUDE_DIRS "")
endif(R_FOUND)




