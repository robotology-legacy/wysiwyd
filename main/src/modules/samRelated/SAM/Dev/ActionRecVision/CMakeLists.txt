cmake_minimum_required(VERSION 2.8.9)


SET(projectName motionMap)
project($projectName)

find_package( OpenCV REQUIRED )
find_package( YARP REQUIRED )

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_executable( ${projectName} ${folder_source} ${folder_header})

target_link_libraries( ${projectName} ${OpenCV_LIBS} ${YARP_LIBRARIES})


