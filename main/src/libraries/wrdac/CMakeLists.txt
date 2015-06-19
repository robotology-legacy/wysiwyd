# Copyright: (C) 2014 EFAA Consortium
# Authors: Stephane Lallee
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME wrdac)
project(${PROJECTNAME})

set(folder_source src/action.cpp
                  src/adjective.cpp
                  src/agent.cpp
                  src/animation.cpp
                  src/entity.cpp           
                  src/functions.cpp
                  src/icubClient.cpp
                  src/object.cpp
                  src/opcClient.cpp
                  src/opcEars.cpp          
                  src/opcSave.cpp
                  src/relation.cpp
                  src/representations.cpp
                  src/rtObject.cpp)

set(folder_header include/wrdac/functions.h
                  include/wrdac/helpers.h
                  include/wrdac/tags.h
                  include/wrdac/knowledge/action.h         
                  include/wrdac/knowledge/adjective.h      
                  include/wrdac/knowledge/agent.h          
                  include/wrdac/knowledge/conjugator.h     
                  include/wrdac/knowledge/entity.h         
                  include/wrdac/knowledge/object.h         
                  include/wrdac/knowledge/relation.h       
                  include/wrdac/knowledge/representations.h
                  include/wrdac/knowledge/rtObject.h
                  include/wrdac/clients/animation.h 
                  include/wrdac/clients/clients.h   
                  include/wrdac/clients/icubClient.h
                  include/wrdac/clients/opcClient.h 
                  include/wrdac/clients/opcEars.h   
                  include/wrdac/clients/opcSave.h
                  include/wrdac/subsystems/all.h                       
                  include/wrdac/subsystems/subSystem.h                 
                  include/wrdac/subsystems/subSystem_ABM.h             
                  include/wrdac/subsystems/subSystem_ARE.h             
                  include/wrdac/subsystems/subSystem_attention.h       
                  include/wrdac/subsystems/subSystem_facialExpression.h
                  include/wrdac/subsystems/subSystem_iKart.h           
                  include/wrdac/subsystems/subSystem_postures.h        
                  include/wrdac/subsystems/subSystem_reactable.h       
                  include/wrdac/subsystems/subSystem_recog.h           
                  include/wrdac/subsystems/subSystem_slidingCtrl.h     
                  include/wrdac/subsystems/subSystem_speech.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wysiwydIDLClients_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
if(WYSIWYD_USE_IQR)
   add_library(${PROJECTNAME} SHARED ${folder_header} ${folder_source})
else()
   add_library(${PROJECTNAME} ${folder_header} ${folder_source})
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER wrDAC)

set_target_properties(${PROJECTNAME} PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} wysiwydIDLClients)

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${wysiwydIDLClients_INCLUDE_DIRS}
                                   DESTINATION include/wysiwyd/${PROJECTNAME}
                                   FILES_WITH_PATH ${folder_header}
                                   PATH_TO_EXCLUDE include/wrdac)

