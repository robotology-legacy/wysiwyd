# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Uriel Martinez
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
#set(PYTHON_ACTIONS demo_actions.py)
#set(PYTHON_INTERACTION demo_interaction.py)

install(PROGRAMS "$ENV{WYSIWYD_ROOT}/main/src/modules/samRelated/SAM/SAM_Demos/demo_actions.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_EXECUTE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE)
yarp_install(PROGRAMS "$ENV{WYSIWYD_ROOT}/main/src/modules/samRelated/SAM/SAM_Demos/demo_actions.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_EXECUTE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE)


install(PROGRAMS "$ENV{WYSIWYD_ROOT}/main/src/modules/samRelated/SAM/SAM_Demos/demo_interactions.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_EXECUTE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE)
yarp_install(PROGRAMS "$ENV{WYSIWYD_ROOT}/main/src/modules/samRelated/SAM/SAM_Demos/demo_interactions.py" DESTINATION bin PERMISSIONS WORLD_READ WORLD_EXECUTE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE)
