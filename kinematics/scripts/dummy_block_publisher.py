"""
A dummy publisher to topic of type 'block.msg' to check block data sensing
and its interaction with kinematics.

Not used in final code, only for troubleshooting
"""

import rospy

from std_msgs.msg import Header, Int16
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from metr4202_msgs.msg import block

def dummy_block_publisher_initiate(waitBool):
    #rospy.loginfo('dummy publisher')
    rospy.init_node('dummy_publisher')
    pub_dummy = rospy.Publisher(
        'priority_block',
        block,
        queue_size=10
    )
    msg = block()
    msg.x = 0
    msg.y = -200
    msg.z = 50
    msg.wait = waitBool
    pub_dummy.publish(msg)

def callback_state(current_state: Int16):
    global state
    state = current_state.data

def main():
    global state
    loop = 0
    # subscribe to state
    sub2 = rospy.Subscriber(
        'metr4202_state',
        Int16,
        callback_state
    )
    dummy_block_publisher_initiate(True)
    while not rospy.is_shutdown():
        loop += 1
        if loop > 30000:
            dummy_block_publisher_initiate(False)
            #rospy.loginfo('Stop waiting ' + str(loop))
            if loop > 60000:
                loop = 0
                rospy.loginfo('reset')
        else:
            dummy_block_publisher_initiate(True)

    
    rospy.spin()

if __name__ == '__main__':
    main()
