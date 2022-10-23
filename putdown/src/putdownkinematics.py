#!/usr/bin/env python3

# Always need this
# from glob import glob
# from msilib.schema import PublishComponent
import rospy

# Import Numpy and Modern_Robotics and pigpio
import numpy as np
import modern_robotics as mr
import pigpio               # this import can generate PWN signal

# Import functions or types
from std_msgs.msg import Header,Int16           #✔
from metr4202_state import current_state        #✔
from sensor_msgs.msg import JointState          #✔
from ximea_color_detect.cpp import ColorRGBA   # necessary import color info #✔


def putdownkinematics() -> JointState: #generate output type which is jointstate// pub JointState
    global pub      # variable  which carry the info of JointState and position
    global state1    # state info from topic 'state'
    # global blockColor 
    """ ROBOT DIMENSION CONSTANTS (mm)"""
    L1 = 50
    L2 = 51
    L3 = 117
    L4 = 95
    L5 = 95
    deltaY = 17
    if state1 == 6:          # if program's state change to 6, start calculating the info where the cube will putdown
        if currentcolor == 0: # different color correspond different zone
            desired_pos = [100, 10, 60] # desired_pos to zone1
        elif currentcolor == 1:
            desired_pos = [80, 120, 60] # desired_pos to zone2
        elif currentcolor ==2:
            desired_pos = [-80, 120, 60] # desired_pos to zone3
        elif currentcolor ==3:
            desired_pos = [-100, 10, 60] # desired_pos to zone4

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

    #rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(msg) # prepare publish msg with type of JointState
    rospy.sleep(4)

def gripper_set(value):           # pub gripper_set
    global gripperPub   # variable contain info of PWN set of gripper
    global statePub     # state info pub to topic "state" to change the whole program to next state  

    # no usage
    # rpi = pigpio.pi()   # use rpi package to control motor
    # rpi.set_mode(18, pigpio.OUTPUT)
    # rpi.set_servo_pulsewidth(18,value)
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position 

    # Create message of type gripset
    # msg = gripperset()          # output msg is gripperset
    # msg.gripperset = (2000)     # msg is 20000
    gripperPub.publish(2000)     # msg published as name "gripperPub"
    
    rospy.sleep(0.5)            # after 0.5s change state to 0, at this moment, the gripper is already open
    statePub = rospy.Publisher('state', Int16)   #statePub is the message which pub to topic 'state'
    statePub.publish(0)  # msg pub to 'state' is 0

def callback_state(current_state: Int16):       # sub current_state
    global state1
    state1 = current_state.data  # receive the state info

def callback_color(ColorRGBA: Int16):       # sub current_state
    global currentcolor
    currentcolor = ColorRGBA
    putdownkinematics()
    
def main ():
    """ Main loop """
    global pub 
    global state1 
    global gripperPub
    global statePub
        # Initialise node with any node name
    rospy.init_node('metr42025_putdownkinematics')

    # creat subscriber of state
    current_state = rospy.Subscriber(
        "metr4202_state", # Topic name
        Int16, # type of info
        callback_state # callback function of state
    )

    # create subscriber of blockColor
    ColorRGBA = rospy.Subscriber(
        "detected_color", # get color info from sensor_msgs
        Int16, # type of info
        callback_color
    )

    # create publisher of JointState
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # create publisher of gripper
    gripperPub = rospy.Publisher(
        'gripper_set',# Topic name
        Int16,# message type
        queue_size=10 # topic size (optional)
    )

    # create publisher of state
    statePub = rospy.Publisher(
        "metr4202_state",  # Topic name of state
        Int16, # message type
        queue_size =10
    )

if __name__ == '__main__':
    main()