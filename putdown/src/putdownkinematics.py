#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Always need this
import rospy

# Import Numpy and Modern_Robotics and pigpio
import numpy as np
import modern_robotics as mr
import pigpio

# Import message types
from std_msgs.msg import Header
from std_msgs.msg import Int16
# from grip_set.msg import putdown_state
from grip_set.msg import gripperset
from sensor_msgs.msg import JointState
from sensor_msgs.msg import blockColor
from sensor_msgs.msg import state
from geometry_msgs.msg import Pose

def state_callback(current_state:Int16):
    global state
    state = current_state.data


def putdownkinematics(pose: Pose) -> JointState:
    global pub
    global state
    
    """ ROBOT DIMENSION CONSTANTS (mm)"""
    L1 = 50
    L2 = 51
    L3 = 117
    L4 = 95
    L5 = 95
    deltaY = 17
    if state1 == 6:
        if blockColor == 0:
            desired_pos = [100, 10, 60]
        elif blockColor == 1:
            desired_pos = [80, 120, 60]
        elif blockColor ==2:
            desired_pos = [-80, 120, 60]
        elif blockColor ==3:
            desired_pos = [-100, 10, 60]
    # subscribe for this
    # neutral pos
    #desired_pos = [0, -100, 100]
    # dropoff 1
    #desired_pos = [100, 10, 60]
    # dropoff 2
    #desired_pos = [80, 120, 60]
    # dropoff 3
    #desired_pos = [-80, 120, 60]
    # dropoff 4
    #desired_pos = [-100, 10, 60]
    # test pos
    #desired_pos = [0, -250, 60]

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
    rospy.sleep(4)

def main():
    """ Main loop """
    global pub
    global sub
    global gripperPub
    global stateSub
    global statePub
    # Initialise node with any node name
    rospy.init_node('metr42025_putdownkinematics')

    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber
    sub = rospy.Subscriber(
        'blockColor', # Topic name
        blockColor, # Message type
        putdownkinematics # Callback function (required)
    )

    # Create subscriber of state
    stateSub = rospy.Subscriber(
        'state', # Topic name
        Int16,  #Message type
        state_callback
    )

    # Pub a msg to let the gripper open and block drop
    gripperPub = rospy.Publisher(
        'gripper_set',# Topic name
        gripperset,# message type
        queue_size=10 # topic size (optional)
    )

    # pub a msg to tell other program that block has dropped and can run other program now

    statePub = 0                 # msg send to topic is 0
    #  msg.data=statePub     
    statePub = rospy.Publisher(
        "state",# Topic name
        Int16,  # msg type
        queue_size=10 
    )

    # Just stops Python from exiting and executes callbacks
    rospy.spin()

def gripper_set(value) -> gripperset:
    global gripperPub
    global statePub
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)
    rpi.set_servo_pulsewidth(18,value)
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position 
    # Create message of type gripset
    msg = gripperset()
    msg.gripperset = (2000)
    gripperPub.publish(msg)
    
    rospy.sleep(0.5)
    statePub = rospy.Publisher('state', Int16)
    statePub.publish(0)


if __name__ == '__main__':
    main()