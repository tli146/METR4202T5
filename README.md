# METR4202T5
ROS repository for robotic arm UQ METR4202
main branch.
run in sequence:
sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null <<<0

sudo pigpiod
roslaunch metr4202_launch metr4202_launch.launch with ROS Noetic to start up program for 1, 2 and 3A
roslaunch metr4202_launch metr4202_launch_3b.launch with ROS Noetic to start up program for 3B 

Required packages:
numpy
scipy
modern_robotics
ximea_ros
Ximea_api
aruco_detect
dynamixel_interface
pigpio

Runtime environment:
ROS 1 Noetic
Ubuntu 20.02
catkin_ws

Code supplied from https://github.com/UQ-METR4202 were used
