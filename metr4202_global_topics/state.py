import rospy

from std_msgs.msg import Int16



def boostState(data):
    global state 
    state = data

    
def main():
    # Initialise node to publish
    rospy.init_node('state_publisher')

    global state 
    state = 1
    # Publish state topic
    pub = rospy.Publisher(
        'metr4202_state',
        Int16,
        
        queue_size=10
    )

    sub = rospy.Subscriber(
        'metr4202_state',
        Int16,
        boostState
    )

    # Set starting state to 1
    pub.publish(1)

    rospy.spin()

if __name__ == '__main__':
    # Run main 
    main()
