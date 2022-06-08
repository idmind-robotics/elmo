cmake_minimum_required(VERSION 3.0.2)
project(elmo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  genmsg
  actionlib_msgs
  dynamic_reconfigure
)


add_message_files(
  FILES
  Colors.msg
  TouchEvent.msg
  PanTilt.msg
)

add_service_files(
  FILES
)

add_action_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/PanTilt.cfg
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS actionlib_msgs
    INCLUDE_DIRS
    LIBRARIES
)