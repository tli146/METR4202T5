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
    msg.y = -150
    msg.z = 100
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
    while not rospy.is_shutdown():
        loop += 1
        if loop > 1000:
            dummy_block_publisher_initiate(False)
        else:
            dummy_block_publisher_initiate(True)
        if state == 5:
            loop = 0
    
    rospy.spin()

if __name__ == '__main__':
    main()
