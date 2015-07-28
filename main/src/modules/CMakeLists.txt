# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#Add here modules without specific dependency
add_subdirectory(attentionRelated)
add_subdirectory(opcRelated)
add_subdirectory(abm)
add_subdirectory(tools)
add_subdirectory(navigationRelated)
#add_subdirectory(reactiveLayer)
add_subdirectory(adaptiveLayer)
add_subdirectory(systemMotor)
add_subdirectory(systemHaptic)
add_subdirectory(systemAuditory)
add_subdirectory(scenarioSpecific)
add_subdirectory(interactionEngine)
add_subdirectory(ppsAllostatic)
add_subdirectory(behaviorRelated)
add_subdirectory(reservoir)
add_subdirectory(touchDetector)

#Add modules with dependencies here
if(WYSIWYD_USE_GTK2)
    add_subdirectory(cvz)
endif()


if(WYSIWYD_USE_kinectWrapper)
    add_subdirectory(humanUnderstanding)
endif()

if(WYSIWYD_USE_OpenCV)
    add_subdirectory(systemVisual)
    add_subdirectory(visionRelated)
endif()

if(WYSIWYD_USE_TUIO)
    add_subdirectory(reactableRelated)
endif()

if(WYSIWYD_USE_OTL AND WYSIWYD_USE_OpenCV)
    add_subdirectory(bodySchema)
endif()

if(WYSIWYD_USE_RTABMap AND WYSIWYD_USE_OpenCV AND WYSIWYD_USE_PCL AND WYSIWYD_USE_Qt5 AND WYSIWYD_USE_kinectWrapper)
    add_subdirectory(perspectiveTaking)
endif()

