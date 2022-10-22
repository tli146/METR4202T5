#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Always need this
import rospy

# Import Numpy and Modern_Robotics
import numpy as np
import modern_robotics as mr

# Import message types
from std_msgs.msg import Header, Int16
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from metr4202_msgs.msg import block

def inverse_kinematics(block_msg: block) -> JointState:
    global pub
    global state
    global state1_wait

    state1_wait = block_msg.wait

    """ ROBOT DIMENSION CONSTANTS (mm)"""
    L1 = 50
    L2 = 51
    L3 = 117
    L4 = 95
    L5 = 95
    deltaY = 17

    # State 1 and wait: robot in initial neutral position
    if state == 1 and block_msg.wait:
        desired_pos = [0, -100, 100]

    # if not wait, move to next state
    elif state == 1 and not block_msg.wait:
        state_pub = rospy.Publisher('state', Int16)
        state_pub.publish(2)

    # State 2: robot in
    elif state == 2:
        desired_pos = [block_msg.x, block_msg.y, block_msg.z]

    # neutral stable pos 
    #desired_pos = [0, -100, 100]
    # dropoff 1
    #desired_pos = [100, 10, 60]
    # dropoff 2
    #desired_pos = [80, 120, 60]
    # dropoff 3
    #desired_pos = [-80, 120, 60]
    # dropoff 4
    #desired_pos = [-100, 10, 60]
    # Show colour
    #desired_pos = [0, -200, 300]
    # test pos
    #desired_pos = [0, -200, 100]
    
    # Subscribe to block information
    #desired_pos = [block_msg.x, block_msg.y, block_msg.z]

    # desired x,y and z (ease of notation)
    dx, dy, dz = desired_pos
    # desired distance to robot
    dr = np.sqrt(dx**2 + dy**2)
    eas = [np.pi/2, 3*np.pi/4, np.pi/4, 0] # End effector angles (with horizontal axis) to iterate through
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

def dummy_block_publisher_initiate(waitBool):
    pub_dummy = rospy.Publisher(
        'priority_block',
        block
    )
    msg = block()
    msg.x = 0
    msg.y = -150
    msg.z = 100
    msg.wait = waitBool
    pub_dummy.publish(msg)

def callback_state(current_state: Int16):
    global state
    state = current_state

def main():
    # Initialise node
    rospy.init_node('invkin_pickup')

    """ Main loop """
    global pub

    # Create publisher to joint states
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # subscribe to block block topic
    sub = rospy.Subscriber(
        'priority_block', # Topic name
        block, # Message type
        inverse_kinematics # Callback function (required)
    )

    # subscribe to state
    sub2 = rospy.Subscriber(
        'state',
        Int16,
        callback_state
    )

    # Stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    frames = 0
    main()
    if frames > 300:
        dummy_block_publisher_initiate(False)
    else:
        dummy_block_publisher_initiate(True)
    frames += 1

