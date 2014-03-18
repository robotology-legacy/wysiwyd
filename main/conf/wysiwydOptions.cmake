# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if(MSVC)
 
   if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
      string(REGEX REPLACE "/W[0-4]" "/W2" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
   else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W2")
   endif()
 
   add_definitions(-D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE)
   set(CMAKE_DEBUG_POSTFIX "d")
 
elseif(CMAKE_COMPILER_IS_GNUCXX)
 
   set(WARNINGS_LIST -Waddress -Wchar-subscripts
   -Wcomment -Wformat -Wmain -Wmissing-braces
   -Wparentheses -Wreorder -Wreturn-type -Wsequence-point
   -Wsign-compare -Wstrict-aliasing -Wstrict-overflow=1 -Wswitch -Wtrigraphs
   -Wuninitialized -Wunused-function -Wunused-label -Wunused-value
   -Wunused-variable -Wvolatile-register-var -Wextra)
 
   foreach(WARNING ${WARNINGS_LIST})
      set(WARNINGS_STRING "${WARNINGS_STRING} ${WARNING}")
   endforeach(WARNING)
 
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNINGS_STRING}")

endif()

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
       set(CMAKE_BUILD_TYPE "Release" CACHE STRING
           "Choose the build type, recommanded options are: Debug or Release" FORCE)
    endif()
    set(WYSIWYD_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${WYSIWYD_BUILD_TYPES})
endif()


