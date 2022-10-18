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

# guess for thetas
theta1_guess = np.arctan2(desired_x, -desired_y)
theta2_guess = np.pi/2 - np.arccos(desired_r/200)
theta3_guess = np.pi - 2*np.arcsin(desired_r/200)
theta4_guess = np.pi - theta2_guess - theta3_guess

def inverse_kinematics():
    """
    Returns joint angles for 
    """
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
    # Desired end effector configuration
    T = np.array([[1,  0,  0, desired_x],
                  [0, -1,  0, desired_y],
                  [0,  0, -1, desired_z],
                  [0,  0,  0, 1]])
    thetalist0 = np.array([theta1_guess, theta2_guess, theta3_guess, theta4_guess])
    eomg = 0.01
    ev = 0.001
    thetalist = mr.IKinBody(Blist, M, T, thetalist0, eomg, ev)
    print(thetalist)
    
    # Check Forward Kin with thetas
    '''thetas, e = thetalist
    newTheta = np.array(thetas)
    desired = mr.FKinBody(M, Blist, newTheta)
    print(desired)'''

def inverse_kin_space():
    Slist = np.array([[0, 0, 1, 0, 0, 0],
                      [1, 0, 0, 0, L1+L2, 0],
                      [1, 0, 0, 0, L1+L2+L3, 0],
                      [1, 0, 0, 0, L1+L2+L3+L4, 0]]).T
    M = np.array([[1, 0, 0, 0],
                  [0, 1, 0, +deltaY],
                  [0, 0, 1, L1+L2+L3+L4+L5],
                  [0, 0, 0, 1]])
    T = np.array([[1,  0,  0, desired_x],
                  [0, -1,  0, desired_y],
                  [0,  0, -1, desired_z],
                  [0,  0,  0, 1]])
    thetalist0 = np.array([theta1_guess, theta2_guess, theta3_guess, theta4_guess])
    eomg = 0.01
    ev = 0.001
    thetalist = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)
    print(thetalist)
#pub_joints = rospy.Publisher('desired_joints', List)
inverse_kinematics()
inverse_kin_space()