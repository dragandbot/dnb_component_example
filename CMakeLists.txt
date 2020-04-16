cmake_minimum_required(VERSION 2.8.3)
project(dnb_component_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  AddTwoNumbers.srv
)

generate_messages( DEPENDENCIES std_msgs )

catkin_package( CATKIN_DEPENDS message_runtime )

include_directories(
  ${catkin_INCLUDE_DIRS}
)
