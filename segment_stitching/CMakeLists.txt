cmake_minimum_required(VERSION 2.8.3)
project(segment_stitching)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag mapping_msgs rosutil sysutil)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection
  CATKIN_DEPENDS roscpp rosbag mapping_msgs
#  DEPENDS visionutil
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(segment_stitching src/segmentStitching.cpp)
target_link_libraries(segment_stitching ${catkin_LIBRARIES} rosutil sysutil)
add_dependencies(segment_stitching ${catkin_EXPORTED_TARGETS})
