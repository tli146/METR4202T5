import rospy

from std_msgs.msg import Header, Int16
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from metr4202_msgs.msg import block

def dummy_block_publisher_initiate(waitBool):
    #rospy.loginfo('dummy publisher initiatied')
    rospy.init_node('dummy_publisher')
    pub_dummy = rospy.Publisher(
        'priority_block',
        block,
        queue_size=10
    )
    msg = block()
    msg.x = 0
    msg.y = -150
    msg.z = 100
    msg.wait = waitBool
    pub_dummy.publish(msg)

def main():
    while not rospy.is_shutdown():
        dummy_block_publisher_initiate(False)
    rospy.spin()

if __name__ == '__main__':
    main()
