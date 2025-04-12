cmake_minimum_required(VERSION 3.2)
project(turtlebot3_mover)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  geometry_msgs
  nav_msgs
  tf
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib move_base_msgs geometry_msgs nav_msgs tf sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlebot3_mover
  src/turtlebot3_mover.cpp
)

# Link the executable with catkin libraries
target_link_libraries(turtlebot3_mover
  ${catkin_LIBRARIES}
)

# Specify dependencies for the executable
add_dependencies(turtlebot3_mover
  ${catkin_EXPORTED_TARGETS}
)


