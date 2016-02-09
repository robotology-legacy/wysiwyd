cmake_minimum_required(VERSION 2.8.3)
project(head_pose_estimation)


#===============================================================================
# FIND PACKAGE DEPENDENCIES
#===============================================================================
# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
  roscpp
  cv_bridge
  rospack
  tf
  cmake_modules )

# Find catkin, ros c++ deps as well as boost
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

#===============================================================================
# CATKIN PACKAGE
#===============================================================================
# This package only creates nodes, so no need to export any information
catkin_package()

#===============================================================================
# INCLUDES
#===============================================================================
include_directories( 
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

#===============================================================================
# COMPILATION
#===============================================================================
#Grab sources for node
file(GLOB SRCS "src/*.h" "src/*.hxx" "src/*.cpp" )

#Add executable
add_executable( ${PROJECT_NAME} ${SRCS} )
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )

#===============================================================================
# INSTALLATION
#===============================================================================
# For binaries...
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# For trees and launch files.
install(DIRECTORY data launch
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}")
