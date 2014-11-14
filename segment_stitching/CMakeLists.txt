cmake_minimum_required(VERSION 2.8.3)
project(segment_stitching)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection
#  CATKIN_DEPENDS roscpp image_transport cv_bridge
#  DEPENDS visionutil
)

include_directories(include ${catkin_INCLUDE_DIRS})
