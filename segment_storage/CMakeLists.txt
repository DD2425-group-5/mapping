cmake_minimum_required(VERSION 2.8.3)
project(segment_storage)

find_package(catkin REQUIRED COMPONENTS roscpp ir_sensors mapping_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection
  CATKIN_DEPENDS roscpp ir_sensors mapping_msgs
#  DEPENDS visionutil
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(segment_storage src/segmentStorage.cpp)
target_link_libraries(segment_storage ${catkin_LIBRARIES})
add_dependencies(segment_storage ${catkin_EXPORTED_TARGETS})
