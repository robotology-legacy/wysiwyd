# Copyright: 2008-2010 RobotCub Consortium
# Author: Stephane Lallee, Maxime Petit, Gregoire Pointeau
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Created:
# PGSQL_INCLUDE_DIRS - Directories to include to use PGSQL
# PGSQL_LIBRARIES    - Default library to link against to use PGSQL
# PGSQL_FOUND        - If false, don't try to use PGSQL

if(NOT PGSQL_DIR)
   set(PGSQL_DIR $ENV{PGSQL_DIR})
endif()

if(PGSQL_DIR)
   find_path(PGSQL_INCLUDE_DIRS libpq-fe.h ${PGSQL_DIR}/include /usr/include/postgresql)
   if(WIN32)
      find_library(PGSQL_LIBRARIES libpq ${PGSQL_DIR}/lib)
   else()
      find_library(PGSQL_LIBRARIES pq ${PGSQL_DIR}/lib)
   endif()

   mark_as_advanced(PGSQL_INCLUDE_DIRS)
   mark_as_advanced(PGSQL_LIBRARIES)
endif()

if(PGSQL_INCLUDE_DIRS AND PGSQL_LIBRARIES)
   set(PGSQL_FOUND TRUE)
else()
   set(PGSQL_FOUND FALSE)
endif()


