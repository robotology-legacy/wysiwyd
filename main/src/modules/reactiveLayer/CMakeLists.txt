# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

install(PROGRAMS "./allostatic_plot.py" "./drive_control_gui.py" "./drive_control_rpc.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)
yarp_install(PROGRAMS "./allostatic_plot.py" "./drive_control_gui.py" "./drive_control_rpc.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE)

add_subdirectory(allostaticController)
add_subdirectory(homeostasis)
add_subdirectory(sensationManager)
add_subdirectory(behaviorManager)
add_subdirectory(goalManager)

