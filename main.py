import rospy

import kinematics.scripts.joint_states_publisher

# Import Numpy and Modern_Robotics
import numpy as np
import modern_robotics as mr

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

def main():
    """ Main Loop """
    kinematics.scripts.joint_states_publisher.main()
    


if __name__ == '__main__':
    main()