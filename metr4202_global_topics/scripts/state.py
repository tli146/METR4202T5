import rospy

from std_msgs.msg import Int16



ros_rate = 2

class StateManager:

    def state_change(self, data):
        self.state = data

    def __init__(self):


        self.state = 1

        self.pub = rospy.Publisher(
            'metr4202_state',
            Int16,
            queue_size=1
        )

        self.sub = rospy.Subscriber('metr4202_state', Int16, self.state_change)

    

    
    

    

if __name__ == '__main__':
    # Run main 
    rospy.init_node('state_manager')
    state_manager = StateManager()
    rate = rospy.Rate(ros_rate)

    while not rospy.is_shutdown():
        state_manager.pub.publish(state_manager.state)

        rate.sleep()
