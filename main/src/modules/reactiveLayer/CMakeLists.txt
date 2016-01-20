# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Maxime Petit
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

install(PROGRAMS "./allostatic_plot.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_EXECUTE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE)

add_subdirectory(allostaticController)
add_subdirectory(homeostasis)
add_subdirectory(sensationManager)
add_subdirectory(behaviorManager)
