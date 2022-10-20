#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Funny code
import random

# Always need this
import rospy

# Import Numpy and Modern_Robotics
import numpy as np
import modern_robotics as mr

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    
    """ ROBOT DIMENSION CONSTANTS (mm)"""
    L1 = 50
    L2 = 51
    L3 = 117
    L4 = 95
    L5 = 95
    deltaY = 17

    # subscribe for this
    desired_pos = [100, 0, 90]
    desired_x, desired_y, desired_z = desired_pos
    desired_r = np.sqrt(desired_x**2 + desired_y**2)

    # desired x,y and z (ease of notation)
    dx, dy, dz = desired_pos
    # desired distance to robot
    dr = desired_r
    eas = [np.pi/2, 3*np.pi/4, np.pi/4] # End effector angles (with horizontal axis) to iterate through
    ea = np.pi/2
    # Iterate through list of end angles (ideally want pi/2 unless out of reach)
    for angle in eas:
        # cos theta_3
        ctheta3 = ((dr - L5*np.cos(angle) - deltaY*np.sin(angle))**2 + (dz+L5*np.sin(angle)-deltaY*np.cos(angle)-L1-L2)**2 - L3**2 - L4**2) \
            / (2*L3*L4)
        # Check if position is within reach
        if 1-(ctheta3)**2 >= 0:
            ea = angle
            break

    # Calculate joint angles
    theta3 = np.arctan2(np.sqrt(1-(ctheta3)**2), ctheta3)
    theta2 = np.arctan2(dr - L5*np.cos(ea) - deltaY*np.sin(ea), (dz+L5*np.sin(ea)-deltaY*np.cos(ea)-L1-L2)) - \
        np.arctan2(L4*np.sin(theta3), L3+L4*np.cos(theta3))
    theta4 = np.pi/2 + ea - theta2 - theta3
    theta1 = np.arctan2(dx, -dy)
    thetalist = [theta1,theta2,theta3,theta4]

    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    # Set angles of the robot
    msg.position = [
        thetalist[0],
        -thetalist[1],
        -thetalist[2],
        thetalist[3]

    ]

    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(msg)

def main():
    global pub
    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber
    sub = rospy.Subscriber(
        'desired_pose', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('metr42025')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()