#!/usr/bin/env python3
import pigpio
import rospy

from std_msgs.msg import Int16

def gripper_set(value):
    """
    Sets the servo value

    :param value:   the value to set servo (2000 for open and 1500 for closed)
    """
    global rpi
    rpi.set_servo_pulsewidth(18,value)

def callback(state: Int16):
    """
    Callback for state subscriber. When state is updated, checks if it
    has new data and whether any new servo assignment is needed

    :param state:   Int16 value of current state
    """
    rospy.loginfo('Callback received') # for troubleshooting
    global state_previous # global variable for the previous state
    # Check if state hasn't changed
    if state_previous != state:
        # Open gripper on given states
        if state.data == 1 or state.data == 2 or state.data == 22:
            gripper_set(2000) # Open
        # Close gripper on given state
        elif state.data == 5 or state.data == 25:
            gripper_set(1500) # Close
    state_previous = state

def main():
    """
    Main function to execute
    """
    # Initiate node and subsriber
    rospy.init_node('gripper_listener')
    sub = rospy.Subscriber('metr4202_state', Int16, callback)
    rospy.spin() # ensure callbacks executed

if __name__ == "__main__":
    # Initiate global previous state
    global state_previous
    global rpi
    rpi = pigpio.pi()
    
    rpi.set_mode(18, pigpio.OUTPUT)
    
    state_previous = 0
    main()