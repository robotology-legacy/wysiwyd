# Copyright: (C) 2014 Marwin SORCE
# Inserm U846
# Try to find naoqi c++ sdk for NAO robot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

# - Try to find NaoQI
# Once done this will define
#  NaoQI_FOUND - System has NaoQI C++ sdk
#  NaoQI_INCLUDE_DIRS - The NaoQI include directories
#  NaoQI_LIBRARIES - The libraries needed to use NaoQI
#  NaoQI_DIR - Directory where NaoQI was found (can be set by user to force 
#               CMake to look in a particular directory)


SET(NaoQI_ROOT_DIR "$ENV{NaoQI_DIR}")
MESSAGE( STATUS "NaoQI_DIR:       " ${NaoQI_ROOT_DIR})
SET(NaoQI_LIB_DIR "${NaoQI_ROOT_DIR}/lib")
SET(NaoQI_INCLUDE_DIR "${NaoQI_ROOT_DIR}/include")

SET(NaoQI_LIBRARIES 
	"${NaoQI_LIB_DIR}/qi.lib"
	"${NaoQI_LIB_DIR}/alaudio.lib"
	"${NaoQI_LIB_DIR}/alcommon.lib"
	"${NaoQI_LIB_DIR}/alerror.lib"
	"${NaoQI_LIB_DIR}/alextractor.lib"
	"${NaoQI_LIB_DIR}/alfile.lib"
	"${NaoQI_LIB_DIR}/allauncher.lib"
	"${NaoQI_LIB_DIR}/allogremote.lib"
	"${NaoQI_LIB_DIR}/almath.lib"
	"${NaoQI_LIB_DIR}/almathinternal_d.lib"
	"${NaoQI_LIB_DIR}/almemoryfastaccess.lib"
	"${NaoQI_LIB_DIR}/alparammanager.lib"
	"${NaoQI_LIB_DIR}/alproject.lib"
	"${NaoQI_LIB_DIR}/alproxies.lib"
	"${NaoQI_LIB_DIR}/alremotecall.lib"
	"${NaoQI_LIB_DIR}/alserial.lib"
	"${NaoQI_LIB_DIR}/alsoap.lib"
	"${NaoQI_LIB_DIR}/althread.lib"
	"${NaoQI_LIB_DIR}/altools.lib"
	"${NaoQI_LIB_DIR}/alvalue.lib"
	"${NaoQI_LIB_DIR}/alvision.lib"
	"${NaoQI_LIB_DIR}/libboost_bzip2-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_date_time-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_filesystem-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_prg_exec_monitor-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_program_options-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_python-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_regex-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_serialization-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_signals-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_system-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_thread-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_wserialization-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/libboost_zlib-vc100-mt-1_47.lib"
	"${NaoQI_LIB_DIR}/pthreadVC2.lib"
	"${NaoQI_LIB_DIR}/pthreadVCE2.lib"
	"${NaoQI_LIB_DIR}/rttools.lib"
	"${NaoQI_LIB_DIR}/tinyxml.lib"
	# "${NaoQI_LIB_DIR}/qi_d.lib"
	# "${NaoQI_LIB_DIR}/alaudio_d.lib"
	# "${NaoQI_LIB_DIR}/alcommon_d.lib"
	# "${NaoQI_LIB_DIR}/alerror_d.lib"
	# "${NaoQI_LIB_DIR}/alextractor_d.lib"
	# "${NaoQI_LIB_DIR}/alfile_d.lib"
	# "${NaoQI_LIB_DIR}/allauncher_d.lib"
	# "${NaoQI_LIB_DIR}/allogremote_d.lib"
	# "${NaoQI_LIB_DIR}/almath_d.lib"
	# "${NaoQI_LIB_DIR}/almemoryfastaccess_d.lib"
	# "${NaoQI_LIB_DIR}/alparammanager_d.lib"
	# "${NaoQI_LIB_DIR}/alproject_d.lib"
	# "${NaoQI_LIB_DIR}/alproxies_d.lib"
	# "${NaoQI_LIB_DIR}/alremotecall_d.lib"
	# "${NaoQI_LIB_DIR}/alserial_d.lib"
	# "${NaoQI_LIB_DIR}/alsoap_d.lib"
	# "${NaoQI_LIB_DIR}/althread_d.lib"
	# "${NaoQI_LIB_DIR}/altools_d.lib"
	# "${NaoQI_LIB_DIR}/alvalue_d.lib"
	# "${NaoQI_LIB_DIR}/alvision_d.lib"
	# "${NaoQI_LIB_DIR}/libboost_bzip2-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_date_time-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_filesystem-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_prg_exec_monitor-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_program_options-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_python-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_regex-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_serialization-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_signals-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_system-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_thread-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_wserialization-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/libboost_zlib-vc100-mt-gd-1_47.lib"
	# "${NaoQI_LIB_DIR}/rttools_d.lib"
	# "${NaoQI_LIB_DIR}/tinyxml_d.lib"
)
SET(NaoQI_INCLUDES 
	"${NaoQI_INCLUDE_DIR}"
)

####################   Macro   #######################
MACRO(CHECK_FILES _FILES _DIR)
	SET(_MISSING_FILES)
	FOREACH(_FILE ${${_FILES}})
		IF(NOT EXISTS "${_FILE}")
			SET(NaoQI_FOUND NO)
			get_filename_component(_FILE ${_FILE} NAME)
			SET(_MISSING_FILES "${_MISSING_FILES}${_FILE}, ")
		ENDIF()
	ENDFOREACH()
	IF(_MISSING_FILES)
		MESSAGE(STATUS "In folder \"${${_DIR}}\" not found files: ${_MISSING_FILES}")
		SET(NaoQI_FOUND NO)
	ENDIF()
ENDMACRO(CHECK_FILES)

MACRO(CHECK_DIR _DIR)
	IF(NOT EXISTS "${${_DIR}}")
		#MESSAGE(STATUS "Folder \"${${_DIR}}\" not found.")
		SET(NaoQI_FOUND NO)
	ENDIF()
ENDMACRO(CHECK_DIR)

##################### Checking #######################
MESSAGE(STATUS "Searching NaoQI_SDK.")
SET(NaoQI_FOUND TRUE)

CHECK_DIR(NaoQI_ROOT_DIR)
IF(NaoQI_FOUND)
	CHECK_DIR(NaoQI_LIB_DIR)
	CHECK_DIR(NaoQI_INCLUDE_DIR)
	MESSAGE(STATUS "checking directories for naoqi")
	
	IF(NaoQI_FOUND)
		MESSAGE(STATUS "checking files for naoqi")
		CHECK_FILES(NaoQI_LIBRARIES NaoQI_LIB_DIR)
		CHECK_FILES(NaoQI_INCLUDES NaoQI_INCLUDE_DIR)
	ENDIF()
ENDIF()

MESSAGE(STATUS "NaoQI_FOUND - ${NaoQI_FOUND}.")

if ( NaoQI_FOUND )
  # If NaoQI was found check its root directory
  get_filename_component(NaoQI_NEW_DIR "${NaoQI_INCLUDE_DIR}/../" ABSOLUTE)
endif()

# Allow the user to propose a hint about where to find NaoQI
# Override value with found directory (empty if not found)
set(NaoQI_DIR ${NaoQI_NEW_DIR} CACHE FILEPATH "NaoQI root directory" FORCE)

mark_as_advanced(NaoQI_INCLUDE_DIR NaoQI_LIBRARIES NaoQI_FOUND)
