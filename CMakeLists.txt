cmake_minimum_required(VERSION 3.10.2)
project(actor_lidar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Lidar2D.cfg
)

catkin_package(
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs dynamic_reconfigure
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
