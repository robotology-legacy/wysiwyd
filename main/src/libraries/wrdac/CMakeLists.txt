# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stephane Lallee
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(wrdac)

set(folder_source src/functions.cpp
                  src/knowledge/action.cpp
                  src/knowledge/adjective.cpp
                  src/knowledge/agent.cpp
                  src/knowledge/bodypart.cpp
                  src/knowledge/entity.cpp
                  src/knowledge/object.cpp
                  src/knowledge/relation.cpp
                  src/knowledge/rtObject.cpp
                  src/subsystems/subSystem_ABM.cpp
                  src/subsystems/subSystem_agentDetector.cpp
                  src/subsystems/subSystem_attention.cpp
                  src/subsystems/subSystem_ARE.cpp
                  src/subsystems/subSystem_babbling.cpp
                  src/subsystems/subSystem_facialExpression.cpp
                  src/subsystems/subSystem_iol2opc.cpp
                  src/subsystems/subSystem_iKart.cpp
                  src/subsystems/subSystem_postures.cpp
                  src/subsystems/subSystem_reactable.cpp
                  src/subsystems/subSystem_recog.cpp
                  src/subsystems/subSystem_LRH.cpp
                  src/subsystems/subSystem_slidingCtrl.cpp
                  src/subsystems/subSystem_speech.cpp
                  src/subsystems/subSystem_KARMA.cpp
                  src/clients/animation.cpp
                  src/clients/icubClient.cpp
                  src/clients/opcClient.cpp
                  src/clients/opcEars.cpp
                  src/clients/opcSave.cpp)


set(folder_header include/wrdac/functions.h
                  include/wrdac/helpers.h
                  include/wrdac/tags.h
                  include/wrdac/knowledge/action.h         
                  include/wrdac/knowledge/adjective.h      
                  include/wrdac/knowledge/agent.h
                  include/wrdac/knowledge/bodypart.h
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
                  include/wrdac/subsystems/subSystem_agentDetector.h
                  include/wrdac/subsystems/subSystem_ARE.h
                  include/wrdac/subsystems/subSystem_attention.h
                  include/wrdac/subsystems/subSystem_babbling.h
                  include/wrdac/subsystems/subSystem_facialExpression.h
                  include/wrdac/subsystems/subSystem_iol2opc.h
                  include/wrdac/subsystems/subSystem_iKart.h           
                  include/wrdac/subsystems/subSystem_postures.h        
                  include/wrdac/subsystems/subSystem_reactable.h       
                  include/wrdac/subsystems/subSystem_recog.h           
                  include/wrdac/subsystems/subSystem_LRH.h           
                  include/wrdac/subsystems/subSystem_slidingCtrl.h     
                  include/wrdac/subsystems/subSystem_speech.h
		  include/wrdac/subsystems/subSystem_KARMA.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${wysiwydIDLClients_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${folder_header} ${folder_source})

if(MSVC)
  target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER wrDAC)
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "d")
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} wysiwydIDLClients)

icub_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${wysiwydIDLClients_INCLUDE_DIRS}
                                    DESTINATION include/${PROJECT_NAME}
                                    FILES_WITH_PATH ${folder_header}
                                    PATH_TO_EXCLUDE include/wrdac)

