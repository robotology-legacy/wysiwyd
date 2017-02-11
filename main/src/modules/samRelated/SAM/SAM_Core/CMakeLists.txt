# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Uriel Martinez
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
set(superVisor samSupervisor.py)
set(optimiser samOptimiser.py)
set(train trainSAMModel.py)
set(interaction interactionSAMModel.py)


add_custom_target(samSupervisor_build ALL)
add_custom_command(TARGET samSupervisor_build POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${superVisor} ${CMAKE_BINARY_DIR}/bin
                   COMMENT "Copying ${superVisor} to ${CMAKE_BINARY_DIR}/bin")

install(PROGRAMS ${superVisor} DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)
yarp_install(PROGRAMS ${superVisor} DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)

add_custom_target(samOptimiser_build ALL)
add_custom_command(TARGET samOptimiser_build POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${optimiser} ${CMAKE_BINARY_DIR}/bin
                   COMMENT "Copying ${optimiser} to ${CMAKE_BINARY_DIR}/bin")

install(PROGRAMS ${optimiser} DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)
yarp_install(PROGRAMS ${optimiser} DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)

add_custom_target(samTrain_build ALL)
add_custom_command(TARGET samTrain_build POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${train} ${CMAKE_BINARY_DIR}/bin
                   COMMENT "Copying ${train} to ${CMAKE_BINARY_DIR}/bin")

install(PROGRAMS ${train} DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)
yarp_install(PROGRAMS ${train} DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)

add_custom_target(samInteraction_build ALL)
add_custom_command(TARGET samInteraction_build POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${interaction} ${CMAKE_BINARY_DIR}/bin
                   COMMENT "Copying ${interaction} to ${CMAKE_BINARY_DIR}/bin")

install(PROGRAMS ${interaction} DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)
yarp_install(PROGRAMS ${interaction} DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)
