import rospy

from std_msgs.msg import Int16

def main():
    # Publish state topic
    pub = rospy.Publisher(
        'state',
        Int16
    )

    # Set starting state to 1
    pub.publish(1)

if __name__ == '__main__':
    # Run main once
    i = True
    if i:
        main()
        i = False