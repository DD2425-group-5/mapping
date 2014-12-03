cmake_minimum_required(VERSION 2.8.3)
project(mapping_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs hardware_msgs geometry_msgs vision_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  SegmentPoint.msg
  MapSegment.msg
  Line.msg
  LineVector.msg
  SegmentLineVector.msg
  Object.msg
  ObjectVector.msg
  SegmentObjectVector.msg
  StitchingResults.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  hardware_msgs
  geometry_msgs
  vision_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection
  CATKIN_DEPENDS std_msgs hardware_msgs message_runtime geometry_msgs vision_msgs
#  DEPENDS visionutil
)

include_directories(include ${catkin_INCLUDE_DIRS})
