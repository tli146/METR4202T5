
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   fiducial_msgs
   message_generation
   tf
)

add_message_files(
   FILES
   block.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs
)
catkin_install_python(PROGRAMS src/detect_block.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)