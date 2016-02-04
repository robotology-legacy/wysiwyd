cmake_minimum_required(VERSION 2.8.3)
project(perspective_taking_python)

find_package(catkin REQUIRED COMPONENTS
  tf
  rospy
  message_generation
  std_msgs
  geometry_msgs
)

#catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CircleEquation.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perspective_taking_python
#  CATKIN_DEPENDS message_generation rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


