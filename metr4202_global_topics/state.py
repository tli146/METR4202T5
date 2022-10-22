import rospy

from std_msgs.msg import Int16

def main():
    # Initialise node to publish
    rospy.init_node('state_publisher')

    # Publish state topic
    pub = rospy.Publisher(
        'metr4202_state',
        Int16,
        queue_size=10
    )

    # Set starting state to 1
    while not rospy.is_shutdown():
        pub.publish(2)

    rospy.spin()

if __name__ == '__main__':
    # Run main 
    main()
