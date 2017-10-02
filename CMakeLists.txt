cmake_minimum_required(VERSION 2.8.3)
project(hri_scenario)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  WrenchArray.msg
)

add_service_files(
  FILES
  setCoMVelDes.srv
  stopWalk.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

