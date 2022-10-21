#!/usr/bin/env python3
import pigpio

def gripper_set(value):
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)
    rpi.set_servo_pulsewidth(18,value)
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position 

if __name__ == "__main__":
    gripper_set(1500)
    