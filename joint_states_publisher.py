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


# Your inverse kinematics function
# This one doesn't actually do it though...
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
    desired_pos = [0, -90, 20]
    desired_x, desired_y, desired_z = desired_pos
    desired_r = np.sqrt(desired_x**2 + desired_y**2)

    # guess for thetas
    theta1_guess = np.arctan2(desired_x, -desired_y)
    theta2_guess = np.pi/2 - np.arccos(desired_r/200)
    theta3_guess = np.pi - 2*np.arcsin(desired_r/200)
    theta4_guess = np.pi - theta2_guess - theta3_guess

    """
    Perform inverse kinematics using mr 
    """
    # List of screws
    Blist = np.array([[0, 0, 1, -deltaY, 0, 0],
                      [1, 0, 0, 0, -L3-L4-L5, deltaY],
                      [1, 0, 0, 0, -L4-L5, deltaY],
                      [1, 0, 0, 0, -L5, deltaY]]).T
    # Home configuration
    M = np.array([[1, 0, 0, 0],
                  [0, 1, 0, deltaY],
                  [0, 0, 1, L1+L2+L3+L4+L5],
                  [0, 0, 0, 1]])
    # Desired end effector configuration
    T = np.array([[1,  0,  0, desired_x],
                  [0, -1,  0, desired_y],
                  [0,  0, -1, desired_z],
                  [0,  0,  0, 1]])
    thetalist0 = np.array([theta1_guess, theta2_guess, theta3_guess, theta4_guess])
    eomg = 0.01
    ev = 0.001
    thetalist = mr.IKinBody(Blist, M, T, thetalist0, eomg, ev)

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
        thetalist[1],
        thetalist[2],
        -thetalist[3]
    ]

    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(dummy_joint_states())

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
    rospy.init_node('metr4202_w7_prac')

    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()