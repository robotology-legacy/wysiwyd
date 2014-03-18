# Copyright: (C) 2011 EFAA Consortium
# Authors: Giorgio Metta, Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# BE VERY CAREFUL: ORDER DOES MATTER!
add_subdirectory(efaaHelpers)
add_subdirectory(d4cActions)

if(EFAA_USE_KinectSDK OR EFAA_USE_OpenNI)
   if (EFAA_USE_KinectSDK AND EFAA_USE_OpenNI)
      set(KINECT_DRIVER_USE_SDK TRUE CACHE BOOL "use sdk instead of OpenNI")
   endif (EFAA_USE_KinectSDK AND EFAA_USE_OpenNI)
endif(EFAA_USE_KinectSDK OR EFAA_USE_OpenNI)
add_subdirectory(kinectWrapper)
add_subdirectory(hammerHelper)
IF(EFAA_FACE_RECOGNITION_AVAILABLE)
   add_subdirectory(faceRecognition)
ENDIF(EFAA_FACE_RECOGNITION_AVAILABLE)

