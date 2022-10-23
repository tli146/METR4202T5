#!/usr/bin/env python3
"""
This script obtains the desired position from block.msg and 
publishes a set of joint states to the dynamixel controller.
Also handles the state transitions since it is run at a high frequency
"""

# Always need this
import rospy

# Import Numpy and Modern_Robotics
import numpy as np
import modern_robotics as mr

# Import message types
from std_msgs.msg import Header, Int16, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from metr4202_msgs.msg import block

global priority_block
ros_rate = 10

def inverse_kinematics(desired_pos):

    rospy.loginfo('Inverse_Kinematics ' + str(desired_pos))

    """ ROBOT DIMENSION CONSTANTS (mm)"""
    L1 = 50
    L2 = 51
    L3 = 117
    L4 = 95
    L5 = 95
    deltaY = 17

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

    # desired x,y and z (ease of notation)
    dx, dy, dz = desired_pos

    # desired distance to robot (radius)
    dr = np.sqrt(dx**2 + dy**2)

    '''# change in z dependent on r
    dz = dz + 0.1*dr

    # change in x and y dependent on r
    dx = dx*(1+0.001*dr)
    dy = dy*(1+0.001*dr)'''

    # Iterate through list of end angles (ideally want pi/2 unless out of reach)
    eas = [np.pi/3, 0, -np.pi/3] # End effector angles (with horizontal axis) to iterate through
    ea = np.pi/2
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
    return thetalist

   

class Joint_Handler:

    def __init__(self):
        self.block_x = 0
        self.block_y = 0
        self.block_z = 0
        self.frames = 0
        self.block_msg = block()
        self.block_msg.wait = True
        self.state = 0

        # Create publisher to joint states
        self.pub = rospy.Publisher(
            'desired_joint_states', # Topic name
            JointState, # Message type
            queue_size=10 # Topic size (optional)
        )

        self.state_pub = rospy.Publisher(
            'metr4202_state', 
            Int16)

        self.publish_message = rospy.Publisher(
            'metr4202_message',
            String
        )  

        # subscribe to block block topic
        self.sub = rospy.Subscriber(
            'priority_block', # Topic name
            block, # Message type
            self.callback_block # Callback function (required)
        )

        # subscribe to the current state
        self.sub2 = rospy.Subscriber(
            'metr4202_state',
            Int16,
            self.callback_state
        )


    def callback_block(self, block_msg: block):
        self.block_msg = block_msg

    def callback_state(self,current_state: Int16):
        self.state = current_state.data


    def loop_joint_state(self):
        ''' Handles what position we want from the state and publishes 
            This will be constantly called as 'priority_block' is changed: won't always use
            the block coords though. Needs priority_block constantly updated '''

        # Copy of block's coords (so it remembers)
        block_x = self.block_x
        block_y = self.block_y
        block_z = self.block_z
        state = self.state
        calibrating = False

        sleep = 0
        desired_pos = [0, -100, 100]
        block_msg = self.block_msg
        thetalist = [0,0,0,0]
        
        # State 0: moving to calibration
        if state == 0:
            thetalist = [0, 2*np.pi/3, -np.pi/6, 0]
            sleep = 3
            self.state_pub.publish(11)
            calibrating = True
        # State 11: intermediate calibration stage
        if state == 11:
            thetalist = [0, 2*np.pi/3, -np.pi/6, 0]
            calibrating = True
            self.state_pub.publish(10)
        # State 10: calibration stage
        if state == 10:
            thetalist = [0, 2*np.pi/3, -np.pi/6, 0]
            calibrating = True
        # State 1 and wait: robot in initial neutral position
        if state == 1 and block_msg.wait:
            desired_pos = [0, -100, 100]
            sleep = 1
            self.publish_message.publish( "1 and wait") 
        # if not wait, move to next state
        elif state == 1 and not block_msg.wait:

            self.block_x = -block_msg.x
            self.block_y = block_msg.y
            self.block_z = block_msg.z

            self.state_pub.publish(2)
            self.publish_message.publish( "1 and not wait") 
            sleep = 1
            if block_x > 20:
                block_x = block_x + 20
            if block_y < -150:
                block_y = block_y + 70
                block_z = block_z + 20
                sleep = 5
        # State 2: robot moving to above the block
        elif state == 2:
            desired_pos = [block_x, block_y, 70]
            self.state_pub.publish(3)
            sleep = 1
        # State 3: robot lowering on block
        elif state == 3:
            desired_pos = [block_x, block_y, 50]
            self.state_pub.publish(4)
            sleep = 1
        # State 4: gripper grabbing block
        elif state == 4:
            desired_pos = [block_x, block_y, 50]
            self.state_pub.publish(5)
            sleep = 1
        # State 5: robot showing block to camera
        elif state == 5:
            desired_pos = [0, -180, 330]
            self.state_pub.publish(6)
            sleep = 4
        # State 6: robot moving to dropoff
        elif state == 6:
            desired_pos = [80, 120, 60]
            self.state_pub.publish(7)
            sleep = 2.5
        # State 7: robot releasing block
        elif state == 7:
            desired_pos = [80, 120, 60]
            self.state_pub.publish(8)
        # State 8: back to home position
        elif state == 8:
            desired_pos = [0, -100, 100]
            self.state_pub.publish(1)
            sleep = 3


        # Perform inverse kinematics for desired position
        

        if not calibrating:
            thetalist = inverse_kinematics(desired_pos)

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

        #rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{msg.position}\nrot:\n{msg.orientation}\n]')
        rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{msg.position}')
        self.pub.publish(msg)
        if not sleep == 0:
            rospy.sleep(sleep)









if __name__ == '__main__':
    rospy.init_node('invkin_pickup')
    


    joint_handler = Joint_Handler()
    rate = rospy.Rate(ros_rate)

    while not rospy.is_shutdown():
        joint_handler.loop_joint_state()
        rate.sleep()

    

