#!/usr/bin/env python3
import pigpio

if __name__ == "__main__":
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)
    rpi.set_servo_pulsewidth(18,2000) 
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position