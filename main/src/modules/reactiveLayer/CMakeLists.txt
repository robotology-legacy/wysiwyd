# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_custom_target(allostatic_plot_build ALL)
add_custom_command(TARGET allostatic_plot_build POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/allostatic_plot.py ${CMAKE_BINARY_DIR}/bin
                   COMMENT "Copying allostatic_plot.py to ${CMAKE_BINARY_DIR}/bin")
add_custom_target(drive_control_gui_build ALL)
add_custom_command(TARGET drive_control_gui_build POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/drive_control_gui.py ${CMAKE_BINARY_DIR}/bin
                   COMMENT "Copying drive_control_gui.py to ${CMAKE_BINARY_DIR}/bin")
add_custom_target(drive_control_rpc_build ALL)
add_custom_command(TARGET drive_control_rpc_build POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/drive_control_rpc.py ${CMAKE_BINARY_DIR}/bin
                   COMMENT "Copying drive_control_rpc.py to ${CMAKE_BINARY_DIR}/bin")

install(PROGRAMS "./allostatic_plot.py" "./drive_control_gui.py" "./drive_control_rpc.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)
yarp_install(PROGRAMS "./allostatic_plot.py" "./drive_control_gui.py" "./drive_control_rpc.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)

add_subdirectory(allostaticController)
add_subdirectory(homeostasis)
add_subdirectory(sensationManager)
add_subdirectory(behaviorManager)
