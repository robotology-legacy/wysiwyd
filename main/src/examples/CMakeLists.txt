# Copyright: (C) 2011 EFAA Consortium
# Authors: Lorenzo Natale, Giorgio Metta
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#add_subdirectory(demoModule)
#add_subdirectory(efaa-lib)
add_subdirectory(worldExample)
add_subdirectory(faceEmotionsExample)
add_subdirectory(icubClientExample)
add_subdirectory(animatorExample)

#if(EFAA_USE_KinectSDK OR EFAA_USE_OpenNI)
	add_subdirectory(kinectClient)
#lendif(EFAA_USE_KinectSDK OR EFAA_USE_OpenNI)
