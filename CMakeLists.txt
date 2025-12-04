cmake_minimum_required(VERSION 3.0.2)
project(robot4ws_trajectory_tracking)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  gazebo_msgs
  tf2
  robot4ws_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(trajectory_tracker_node src/trajectory_tracker_node.cpp)
target_link_libraries(trajectory_tracker_node ${catkin_LIBRARIES})

install(TARGETS trajectory_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
