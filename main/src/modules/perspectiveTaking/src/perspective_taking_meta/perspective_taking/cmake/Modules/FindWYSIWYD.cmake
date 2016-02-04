if(NOT WYSIWYD_DIR)
   set(WYSIWYD_DIR $ENV{WYSIWYD_DIR})
endif()

if(WYSIWYD_DIR)
    find_path(WYSIWYD_INCLUDE_DIRS cvzMmcm_IDL.h ${WYSIWYD_DIR}/include)
    find_library(WYSIWYD_LIBRARIES wrdac ${WYSIWYD_DIR}/lib)

    mark_as_advanced(WYSIWYD_INCLUDE_DIRS)
    mark_as_advanced(WYSIWYD_LIBRARIES)
endif()

if(WYSIWYD_INCLUDE_DIRS AND WYSIWYD_LIBRARIES)
   set(WYSIWYD_FOUND TRUE)
else()
   set(WYSIWYD_FOUND FALSE)
endif()

