#!/usr/bin/env python3
import pigpio
import rospy

from std_msgs.msg import Int16

def gripper_set(value):
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)
    rpi.set_servo_pulsewidth(18,value)
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position 

def callback(state: Int16):
    if state.data == 1:
        gripper_set(2000) # Open
    elif state.data == 2:
        gripper_set(1500) # Close

def main():
    rospy.init_node('gripper_listener')
    sub = rospy.Subscriber('metr4202_state', Int16, callback)
    rospy.spin()

if __name__ == "__main__":
    main()