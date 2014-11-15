cmake_minimum_required(VERSION 2.8.3)
project(logger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES logger
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(logger src/logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES})
add_dependencies(logger ${catkin_EXPORTED_TARGETS})