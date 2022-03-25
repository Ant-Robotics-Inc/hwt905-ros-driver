cmake_minimum_required(VERSION 3.0.2)
project(inclinometer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
)

catkin_package(
 CATKIN_DEPENDS std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hwt905_node src/hwt905_node.cpp)
add_dependencies(hwt905_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hwt905_node
  ${catkin_LIBRARIES}
)

add_executable(hwt905_tf_broadcaster_node src/hwt905_tf_broadcaster_node.cpp)
add_dependencies(hwt905_tf_broadcaster_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hwt905_tf_broadcaster_node
  ${catkin_LIBRARIES}
)
