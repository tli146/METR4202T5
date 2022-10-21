import rospy

from geometry_msgs.msg import Pose
from std_msgs.msg import Float32


def main():
    """Mainloop, obtain a user input (Desired pose) and publish to
    Ben's Topic"""
    rospy.init_node("comms_node", anonymous=True)
    posePub = rospy.Publisher("desired_pose", Pose, queue_size=10)
    gripperPub = rospy.Publisher("gripper", Float32, queue_size=10)
    #Develop a series of Pose Messages:
    pose1 = Pose()
    pose1.position.x = 100
    pose1.position.y = 0
    pose1.position.z = 1

    pose1.orientation.x = 1
    pose1.orientation.y = 0
    pose1.orientation.z = 1
    pose1.orientation.w = 1

    gripOpen = Float32()
    gripOpen.data = 2000

    gripClose = Float32()
    gripClose.data = 1500

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        gripperPub.publish(gripOpen)
        rospy.sleep(2)
        posePub.publish(pose1)
        gripperPub.publish(gripClose)
        rospy.sleep(2)


if __name__ == "__main__":
    main()