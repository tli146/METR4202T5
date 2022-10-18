#import rospy
import numpy as np
import modern_robotics as mr
#from std_msgs.msg import Header
#from sensor_msgs.msg import JointState
#from geometry_msgs.msg import Pose


""" Take an array of positions and turn them into a list of joint angles (4 angles) """

# == ROBOT DIMENSION CONSTANTS ==
L1 = 50
L2 = 51
L3 = 117
L4 = 95
L5 = 95
deltaY = 17
# ===============================

# subscribe to topic for this
desired_pos = [0, -250, 20]

desired_x, desired_y, desired_z = desired_pos
desired_r = np.sqrt(desired_x**2 + desired_y**2)
    
def inverse_kin_analytical(end_position):
    # desired x,y and z
    dx, dy, dz = end_position
    # desired distance to robot
    dr = desired_r
    eas = [np.pi/2, 3*np.pi/4, np.pi/4] # End effector angles (with horizontal axis)
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
    thetas = [theta1,theta2,theta3,theta4]
    return thetas


# Use forward kinematics to test positions if wanted
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
    
# Check Forward Kin with thetas
thetas = np.array(inverse_kin_analytical(desired_pos))
desired = mr.FKinBody(M, Blist, thetas)
print(desired)