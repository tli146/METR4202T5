


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  message_generation
   
)
project( metr4202_putdown )
catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs message_runtime
)

add_message_files(
  FILES
  grip_set.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS rospy std_msgs message_runtime)

catkin_install_python(PROGRAMS src/metr4202_putdownkinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)