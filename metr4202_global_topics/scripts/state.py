import rospy

from std_msgs.msg import Int16



ros_rate = 2 # the rate to run this package

class StateManager:
    """ 
    A class which handles which state were in by repetitively and
    asynchronously publishing it for subscribers
    """

    def state_change(self, data):
        """
        Callback function which sets the overall state from given data

        :param data:    the Int16 data of what state to change to
        """
        self.state = data

    def __init__(self):
        self.state = 0 # our current state

        # State publisher
        self.pub = rospy.Publisher(
            'metr4202_state',
            Int16,
            queue_size=1
        )

        # State subscriber
        self.sub = rospy.Subscriber(
            'metr4202_state',
            Int16,
            self.state_change)
    

if __name__ == '__main__':
    # Initiate node
    rospy.init_node('state_manager')

    # Create object of type StateManager
    state_manager = StateManager()

    # Set the rate
    rate = rospy.Rate(ros_rate)

    # Loop through publishing states 
    while not rospy.is_shutdown():
        state_manager.pub.publish(state_manager.state)
        rate.sleep()
