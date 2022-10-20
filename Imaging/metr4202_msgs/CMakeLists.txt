project( metr4202_msgs )
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
catkin_package()
