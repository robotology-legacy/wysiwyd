# Copyright: (C) 2014 EFAA Consortium
# Authors: Stephane Lallee
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME wrdac)
project(${PROJECTNAME})

file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/wrdac/*.h  )
file(GLOB folder_header_knowledge include/wrdac/knowledge/*.h)
file(GLOB folder_header_clients include/wrdac/clients/*.h)
file(GLOB folder_header_subsystems include/wrdac/subsystems/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})
source_group("Header Files: Knowledge" FILES ${folder_header_knowledge})
source_group("Header Files: Clients" FILES ${folder_header_clients})
source_group("Header Files: Subsystems" FILES ${folder_header_subsystems})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${wysiwydIDLClients_INCLUDE_DIRS})
if(WYSIWYD_USE_IQR)
   add_library(${PROJECTNAME} SHARED ${folder_source} ${folder_header} ${folder_header_knowledge} ${folder_header_clients} ${folder_header_subsystems})
else()
   add_library(${PROJECTNAME} ${folder_source} ${folder_header} ${folder_header_knowledge} ${folder_header_clients} ${folder_header_subsystems})
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER wrDAC)

set_target_properties(${PROJECTNAME} PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} wysiwydIDLClients)

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${wysiwydIDLClients_INCLUDE_DIRS}
                                   DESTINATION include/
                                   FILES ${folder_header})

