""" Determines if conditions are met to move to next state and publishes """

'''import rospy

from std_msgs.msg import Int16
from metr4202_msgs.msg import block

def callback(current_state: Int16) -> Int16:
    # Callback of subscriber tests conditions of state change
    STATE CONDITIONS
    if current_state == 1:
        nothing

def main():
    # Publisher to state topic
    pub = rospy.Publisher(
        'state',
        Int16
    )
    # Subscribe to same topic
    sub = rospy.Subscriber(
        'state',
        Int16,
        callback
    )
    # Subscribe to obtain block info
    block_subscriber = rospy.Subscriber('priority_block', block)
    # Set starting state to 1
    pub.publish(1)

if __name__ == '__main__':
    # Run main
    main()'''