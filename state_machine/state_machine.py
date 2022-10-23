""" Determines if conditions are met to move to next state and publishes """

import rospy
import time

from std_msgs.msg import Int16
from metr4202_msgs.msg import block

def callback(state: Int16) -> Int16:
    rospy.loginfo('state callback')
    rospy.loginfo('state = ' + str(state))
    # Callback of subscriber tests conditions of state change
    global block_wait
    global pub
    # State 1 and wait: robot in initial neutral position
    # if not wait, move to next state
    if state.data == 1:
        pub.publish(2)

    # State 2: robot moving to above the block
    elif state.data == 2:
        time.sleep(5)
        pub.publish(3)

    # State 3: robot lowering on block
    elif state.data == 3:
        time.sleep(4)
        pub.publish(4)
    
    # State 4: gripper grabbing block
    elif state.data == 4:
        time.sleep(2)
        pub.publish(5)

    # State 5: robot showing block to camera
    elif state.data == 5:
        time.sleep(5)
        pub.publish(6)

    # State 6: robot showing block to camera
    elif state.data == 6:
        time.sleep(5)
        pub.publish(1)

def callback_block(block_data: block):
    global block_wait
    block_wait = block.wait
    #rospy.loginfo('block data received')

def main():
    global block_wait
    global pub
    # Give block wait some data initially
    block_wait = True

    # Initialise node
    rospy.init_node('state_machine')
    # Publisher to state topic
    pub = rospy.Publisher(
        'metr4202_state',
        Int16,
        queue_size=10
    )
    # Subscribe to same topic
    sub = rospy.Subscriber(
        'metr4202_state',
        Int16,
        callback
    )
    # Subscribe to obtain block info
    #block_subscriber = rospy.Subscriber('priority_block', block, callback_block)
    # Set starting state to 1
    pub.publish(1)
    rospy.spin()

if __name__ == '__main__':
    # Run main
    main()

