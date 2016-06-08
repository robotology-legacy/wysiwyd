cmake_minimum_required(VERSION 2.8.3)
project(perspective_taking_c03)

#===============================================================================
# FIND PACKAGE DEPENDENCIES
#===============================================================================

set( ROS_CXX_DEPENDENCIES
  roscpp
  cv_bridge
  image_transport
  rospack
  tf
  tf_conversions
  cmake_modules
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  perspective_taking
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF()
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

find_package(Eigen REQUIRED)
find_package(VTK REQUIRED)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(WYSIWYD REQUIRED)
find_package(perspective_taking REQUIRED)

include(${VTK_USE_FILE})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES perspective_taking
  CATKIN_DEPENDS cv_bridge roscpp rospack std_msgs tf tf_conversions pcl_ros perspective_taking
  DEPENDS Eigen
)

###########
## Build ##
###########

file(GLOB headers include/${PROJECT_NAME}/*.h)

## Specify additional locations of header files
include_directories(
  include/${PROJECT_NAME}/
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${YARP_INCLUDE_DIRS}
  ${ICUB_INCLUDE_DIRS}
  ${WYSIWYD_INCLUDE_DIRS}
  ${perspective_taking_INCLUDE_DIRS}
)

## Declare executables

add_executable( raytracer src/raytracer.cpp ${headers} )
add_dependencies( raytracer ${catkin_EXPORTED_TARGETS} )
target_link_libraries( raytracer ${catkin_LIBRARIES} ${YARP_LIBRARIES} ${WYSIWYD_LIBRARIES} ${VTK_LIBRARIES} ${perspective_taking_LIBRARIES} )

add_executable( point_cloud_filter src/point_cloud_filter.cpp ${headers} )
add_dependencies( point_cloud_filter ${catkin_EXPORTED_TARGETS} )
target_link_libraries( point_cloud_filter ${catkin_LIBRARIES} ${YARP_LIBRARIES} ${WYSIWYD_LIBRARIES} ${perspective_taking_LIBRARIES} )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS 
    raytracer
    point_cloud_filter
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# For launch files.
install(DIRECTORY launch
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}")
