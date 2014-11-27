cmake_minimum_required(VERSION 2.8.3)
project(map_representation)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs rosutil mapping_msgs maputil)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection 
  CATKIN_DEPENDS roscpp nav_msgs rosutil mapping_msgs maputil
#  DEPENDS visionutil
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(map_representation src/mapRepresentation.cpp)
target_link_libraries(map_representation ${catkin_LIBRARIES} rosutil maputil)
add_dependencies(map_representation ${catkin_EXPORTED_TARGETS})
