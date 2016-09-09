cmake_minimum_required(VERSION 2.8.3)
project(perspective_taking)

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
  pcl_ros
  pcl_conversions
  perspective_taking_python
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
find_package(Eigen REQUIRED)
find_package(VTK REQUIRED)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(WYSIWYD REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(kinectWrapper REQUIRED)

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
  LIBRARIES pt_lib
  CATKIN_DEPENDS cv_bridge roscpp rospack std_msgs tf tf_conversions pcl_ros
  DEPENDS Eigen
  CFG_EXTRAS perspective_taking-extras.cmake
)

###########
## Build ##
###########

file(GLOB headers include/${PROJECT_NAME}/*.h)

## Specify additional locations of header files
include_directories(
  include/${PROJECT_NAME}/
  ${YARP_INCLUDE_DIRS}
  ${ICUB_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${WYSIWYD_INCLUDE_DIRS}
  ${kinectWrapper_INCLUDE_DIRS}
)

## Declare common source files
add_library(pt_lib src/point_cloud_visualizer)
target_link_libraries(pt_lib ${PCL_LIBRARIES})

## Declare executables
add_executable( mental_perspective_transformer src/mental_perspective_transformer.cpp ${headers} )
add_dependencies( mental_perspective_transformer ${catkin_EXPORTED_TARGETS} )
target_link_libraries( mental_perspective_transformer ${catkin_LIBRARIES} pt_lib )

add_executable( visualize_robot_perspective src/visualize_robot_perspective.cpp ${headers} )
add_dependencies( visualize_robot_perspective ${catkin_EXPORTED_TARGETS} )
target_link_libraries( visualize_robot_perspective ${catkin_LIBRARIES} pt_lib )

add_executable( naive_partner_visualizer src/naive_partner_visualizer.cpp ${headers} )
add_dependencies( naive_partner_visualizer ${catkin_EXPORTED_TARGETS} )
target_link_libraries( naive_partner_visualizer ${catkin_LIBRARIES} pt_lib )

add_executable( opc_to_tf src/opc_to_tf.cpp ${headers} )
add_dependencies( opc_to_tf ${catkin_EXPORTED_TARGETS} )
target_link_libraries( opc_to_tf ${catkin_LIBRARIES} ${YARP_LIBRARIES} ${WYSIWYD_LIBRARIES} )

add_executable( gaze_to_tf src/gaze_to_tf.cpp ${headers} )
add_dependencies( gaze_to_tf ${catkin_EXPORTED_TARGETS} )
target_link_libraries( gaze_to_tf ${catkin_LIBRARIES} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} )

add_executable( publish_screenshot src/publish_screenshot.cpp ${headers} )
add_dependencies( publish_screenshot ${catkin_EXPORTED_TARGETS} )
target_link_libraries( publish_screenshot ${catkin_LIBRARIES} pt_lib )

add_executable( camera_to_world_coordinates src/camera_to_world_coordinates.cpp ${headers} )
add_dependencies( camera_to_world_coordinates ${catkin_EXPORTED_TARGETS} )
target_link_libraries( camera_to_world_coordinates ${catkin_LIBRARIES} )

add_executable( spatial_perspective_2 src/spatial_perspective_2.cpp ${headers} )
add_dependencies( spatial_perspective_2 ${catkin_EXPORTED_TARGETS} )
target_link_libraries( spatial_perspective_2 ${catkin_LIBRARIES} ${YARP_LIBRARIES} ${WYSIWYD_LIBRARIES} )

add_executable( point_cloud_transformer src/point_cloud_transformer.cpp ${headers} )
add_dependencies( point_cloud_transformer ${catkin_EXPORTED_TARGETS} )
target_link_libraries( point_cloud_transformer ${catkin_LIBRARIES} )

add_executable( point_cloud_merger src/point_cloud_merger.cpp ${headers} )
add_dependencies( point_cloud_merger ${catkin_EXPORTED_TARGETS} )
target_link_libraries( point_cloud_merger ${catkin_LIBRARIES} )

add_executable( skeleton_visualizer src/skeleton_visualizer.cpp ${headers} )
add_dependencies( skeleton_visualizer ${catkin_EXPORTED_TARGETS} )
target_link_libraries( skeleton_visualizer ${catkin_LIBRARIES} pt_lib )

add_executable( fake_ikart_robotinterface src/fake_ikart_robotinterface ${headers} )
add_dependencies( fake_ikart_robotinterface ${catkin_EXPORTED_TARGETS} )
target_link_libraries( fake_ikart_robotinterface ${catkin_LIBRARIES} ${YARP_LIBRARIES} )

add_executable( depth_to_pointcloud src/depth_to_pointcloud.cpp ${headers} )
add_dependencies( depth_to_pointcloud ${catkin_EXPORTED_TARGETS} )
target_link_libraries( depth_to_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YARP_LIBRARIES} ${kinectWrapper_LIBRARIES} ${WYSIWYD_LIBRARIES} )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
    publish_screenshot
    naive_partner_visualizer
    camera_to_world_coordinates
    mental_perspective_transformer
    visualize_robot_perspective
    opc_to_tf
    gaze_to_tf
    spatial_perspective_2
    point_cloud_transformer
    point_cloud_merger
    skeleton_visualizer
    fake_ikart_robotinterface
    depth_to_pointcloud
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
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
