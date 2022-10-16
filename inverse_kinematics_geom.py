#import rospy
import numpy as np
import modern_robotics as mr
#from std_msgs.msg import Header
#from sensor_msgs.msg import JointState
#from geometry_msgs.msg import Pose


""" Take an array of positions and orientations and publish a list of joint angles (4 angles) """

# Do the inverse kinematics math to return a list of joint angles
# Get the robot to do those angles

""" ROBOT DIMENSION CONSTANTS (mm)"""
L1 = 50
L2 = 51
L3 = 117
L4 = 95
L5 = 95
deltaY = 17

# subscribe for this
desired_pos = [0, -90, 20]
desired_x, desired_y, desired_z = desired_pos
desired_r = np.sqrt(desired_x**2 + desired_y**2)

def inverse_kin_analytical():
    # Ease of notation
    dx, dy, dz = desired_pos
    dr = desired_r
    print(dr)
    # cos theta_3
    ctheta3 = (dr**2 + (dz+L5-L1-L2)**2 - L3**2 - L4**2) / (2*L3*L4)
    print(ctheta3)
    theta3 = np.arctan2(np.sqrt(1-(ctheta3)**2), ctheta3)
    print(theta3)
    theta2 = np.arctan2(dr, (dz+L5-L1-L2)) - np.arctan2(L4*np.sin(theta3), L3+L4*np.cos(theta3))
    print(np.arctan2(dr, dz))
    print(np.arctan2(L4*np.sin(theta3), L3+L4*np.cos(theta3)))
    print(ctheta3)
    print(np.cos(theta3))
    theta4 = np.pi - theta2 - theta3
    theta1 = np.arctan2(dx, -dy)
    print([theta1,theta2,theta3,theta4])
    
def inverse_kin_analytical2():
    # Ease of notation
    dx, dy, dz = desired_pos
    dr = desired_r
    ea = np.pi/2
    # cos theta_3
    ctheta3 = ((dr - L5*np.cos(ea))**2 + (dz+L5*np.sin(ea)-L1-L2)**2 - L3**2 - L4**2) \
        / (2*L3*L4)
    theta3 = np.arctan2(np.sqrt(1-(ctheta3)**2), ctheta3)
    theta2 = np.arctan2(dr - L5*np.cos(ea), (dz+L5*np.sin(ea)-L1-L2)) - \
        np.arctan2(L4*np.sin(theta3), L3+L4*np.cos(theta3))
    theta4 = np.pi/2 + ea - theta2 - theta3
    theta1 = np.arctan2(dx, -dy)
    thetas = [theta1,theta2,theta3,theta4]
    print(thetas)

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
    newTheta = np.array(thetas)
    desired = mr.FKinBody(M, Blist, newTheta)
    print(desired)
    
def inverse_kin_analytical3():
    # Ease of notation
    dx, dy, dz = desired_pos
    dr = desired_r
    ea = np.pi/2
    # cos theta_3
    ctheta3 = ((dr - L5*np.cos(ea))**2 + (dz+L5*np.sin(ea)-L1-L2)**2 - L3**2 - L4**2) \
        / (2*L3*L4)
    theta3 = np.arctan2(np.sqrt(1-(ctheta3)**2), ctheta3)
    theta2 = np.arctan2(dr - L5*np.cos(ea), (dz+L5*np.sin(ea)-L1-L2)) - \
        np.arctan2(L4*np.sin(theta3), L3+L4*np.cos(theta3))
    theta4 = np.pi/2 + ea - theta2 - theta3
    theta1 = np.arctan2(dx, -dy)
    thetas = [theta1,theta2,theta3,theta4]
    print(thetas)

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
    newTheta = np.array(thetas)
    desired = mr.FKinBody(M, Blist, newTheta)
    print(desired)
#pub_joints = rospy.Publisher('desired_joints', List)

#inverse_kin_analytical()
#inverse_kinematics()
inverse_kin_analytical3()