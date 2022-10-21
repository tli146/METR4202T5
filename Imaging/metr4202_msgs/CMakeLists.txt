project( metr4202_msgs )
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

add_message_files(
   FILES
   block.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs
)
catkin_package(
   CATKIN_DEPENDS message_runtime
)
