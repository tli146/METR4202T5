cmake_minimum_required(VERSION 3.16)



find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   fiducial_msgs
   message_generation
   tf
   metr4202_msgs
   
)
project( metr4202_transforms )
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime message_runtime metr4202_msgs
)

catkin_install_python(PROGRAMS src/detect_block.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)