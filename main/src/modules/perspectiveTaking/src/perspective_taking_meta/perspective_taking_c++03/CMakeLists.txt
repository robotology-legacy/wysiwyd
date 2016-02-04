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

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

find_package(Eigen REQUIRED)
find_package(VTK REQUIRED)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(WYSIWYD REQUIRED)
find_package(perspective_taking REQUIRED)

include(${VTK_USE_FILE})

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
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
target_link_libraries( raytracer ${catkin_LIBRARIES} ${YARP_LIBRARIES} ${WYSIWYD_LIBRARIES} ${perspective_taking_LIBRARIES} )

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
