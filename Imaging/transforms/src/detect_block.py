from matplotlib.pyplot import thetagrids
import rospy
import numpy as np
import modern_robotics as mr
from block.msg import Block
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

from geometry_msgs.msg import Transform
from std_msgs.msg import Header, String, int8, int16, bool



class DetectedBlock:
    def __init__(self, id, x,y,z,theta,color ) -> None:
        self.id = id
        self.coordinate = (x,y,z)
        self.theta = theta
        self.color = color
        self.priority = 0
    
    def toMsg(self) -> Block:
        msg = Block()
        Block.id = self.id
        Block.blockColor = self.color
        Block.x = self.coordinate[0]
        Block.y = self.coordinate[1]
        Block.z = self.coordinate[2]
        Block.theta = self.theta
        return Block

    


        


class DetectBlock:

    def __init__(self, serial):
        self.pub =rospy.Publisher(
        'Trx', # Topic name
        Transform, # Message type
        queue_size=10 # Topic size (optional)
        ) 

        self.sub_fiducials_transform = rospy.Subscriber("/fiducial_transforms",
        FiducialTransformArray,
        detection_callback       
        )
        self.calibrated = False

        #set calibration aruco code location
        self.Tr = np.array([
            [1,0,0,40],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,0]
        ])

        self.Trx = np.eye(4)

        self.transformsList = []

    def detection_callback(fiducialTransformArray: FiducialTransformArray):
        self.transformList = fiducialTransformArray.transforms



   



    


if __name__ == '__main__':

    rospy.init_node('detect_block')
    #set frequency to increase performance
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #publish message
        rate.sleep()
    rospy.spin()

