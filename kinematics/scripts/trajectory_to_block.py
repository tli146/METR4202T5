#!/usr/bin/env python3
"""
Publishes a set of joints representative of the trajectory to get to just above the block
"""

import rospy

# Import Numpy and Modern_Robotics
import numpy as np
import modern_robotics as mr

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

def trajectory_generation(pose: Pose) -> JointState:
    """
    Generate the trajectory between two positions. This will use a parabolic trajectory
    (currently is linear) for the euclidean distance and a 5th order polynomial for time
    """
    global pub

    # OBTAIN INITIAL JOINT ANGLES (this will be at position [100, 0, 100])
    thetalist0 = np.array(inverse_kinematics([0, -100, 100]))
    # OBTAIN DESIRED END ANGLES
    thetalistend = np.array(inverse_kinematics([0, -200, 60]))

    # Constants for 5th order polynomial
    a3 = 1.25
    a4 = -15/16
    a5 = 3/16

    # T = 3 secs, 60 FPS
    steps = 3*60
    dt = 1/60
    T = (steps-1)*dt

    # Initialise thetalist desired trajectory
    thetalistd = np.zeros([steps, len(thetalist0)])

    for n in range(steps - 1):
        s = a3*(n*dt)**3 + a4*(n*dt)**4 + a5*(n*dt)**5
        thetalistd[n] = (thetalist0 + s*(thetalistend - thetalist0))

        # Create message of type JointState
        msg = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )
        # Set angles of the robot
        msg.position = [
            thetalistd[n,0],
            -thetalistd[n,1],
            -thetalistd[n,2],
            thetalistd[n,3]
        ]
        rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{msg.position}\nrot:\n{pose.orientation}\n]')
        pub.publish(msg)

def inverse_kinematics(desired_pos):
    """
    Returns joint angles of robot for given position

    :param desired_pos:     3d position list of form [x, y, z]
    """
    global pub
    
    """ ROBOT DIMENSION CONSTANTS (mm)"""
    L1 = 50
    L2 = 51
    L3 = 117
    L4 = 95
    L5 = 95
    deltaY = 17

    # desired x,y and z (ease of notation)
    dx, dy, dz = desired_pos
    # desired distance to robot
    dr = np.sqrt(dx**2 + dy**2)
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
    
    # Slight coordinate changes due to frame assignment
    return [thetalist[0], -thetalist[1], -thetalist[2], thetalist[3]]


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
        trajectory_generation # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('metr4202_w7_prac')

    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()