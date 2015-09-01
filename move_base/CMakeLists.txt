cmake_minimum_required(VERSION 2.8.3)
project(move_base)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  roslaunch
)

roslaunch_add_file_check(launch)

catkin_python_setup()

catkin_package(
CATKIN_DEPENDS rospy geometry_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

