cmake_minimum_required(VERSION 2.8.3)
project(topological_map)

find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    hardware_msgs
                    mapping_msgs
                    geometry_msgs
                    rosutil
                    mathutil
                    sysutil
                    vision_msgs
                    visualization_msgs
                    rosbag)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection
  CATKIN_DEPENDS roscpp sysutil mathutil mapping_msgs geometry_msgs rosutil vision_msgs hardware_msgs visualization_msgs rosbag
#  DEPENDS visionutil
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(topological_map src/topological.cpp)
target_link_libraries(topological_map ${catkin_LIBRARIES} rosutil sysutil mathutil)
add_dependencies(topological_map ${catkin_EXPORTED_TARGETS})
