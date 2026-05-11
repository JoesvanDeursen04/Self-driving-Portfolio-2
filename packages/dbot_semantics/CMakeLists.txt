cmake_minimum_required(VERSION 2.8.3)
project(dbot_semantics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  tf
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    cv_bridge
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/semantic_perception_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
