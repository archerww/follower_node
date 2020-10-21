cmake_minimum_required(VERSION 2.8.3)
project(follower_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  astar_planner
  autoware_msgs
  gnss
  jsk_recognition_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  vector_map
  waypoint_follower
  geometry_msgs
  message_generation
  zr_msgs
  
)
find_package(OpenCV REQUIRED)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
catkin_package(
##  INCLUDE_DIRS include
 ## LIBRARIES follower_node
  CATKIN_DEPENDS astar_planner zr_msgs autoware_msgs gnss jsk_recognition_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf vector_map waypoint_follower
  DEPENDS system_lib
)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)
add_executable(follower_node src/follower_node.cpp)
 target_link_libraries(follower_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  
)
 ##target_link_libraries(follower_node_1  ${OpenCV_LIBS})
 add_dependencies(follower_node autoware_msgs_generate_messages_cpp)
 add_dependencies(follower_node zr_msgs_generate_messages_cpp)
 
