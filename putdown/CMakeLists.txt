cmake_minimum_required(VERSION 3.16)



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  message_generation
  ximea_color
  metr4202_topics
   
)


add_message_files(
  FILES
  grip_set.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
project( metr4202_putdown )

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs message_runtime ximea_color_detect metr4202_state
)


catkin_install_python(PROGRAMS scripts/putdownkinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)