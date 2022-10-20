from matplotlib.pyplot import thetagrids
import rospy
import math
import heapq
from scipy.spatial.transform import Rotation as R
import numpy as np
import modern_robotics as mr
from block.msg import Block
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import Transform
from std_msgs.msg import Header, String, int8, int16, bool

calibration_ID = 10

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

    def _is_valid_operand(self, other):
        return (hasattr(other, "priority"))

    def __eq__(self, other):
        if not self._is_valid_operation(other):
            return NotImplemented
        return self.priority == other.priority

    
    def __lt__(self, other):
        if not self._is_valid_operation(other):
            return NotImplemented
        return self.priority < other.priority

    def __gt__(self, other):
        if not self._is_valid_operation(other):
            return NotImplemented
        return self.priority > other.priority





    


        


class DetectBlock:

    def __init__(self):
        self.pub =rospy.Publisher(
        'priority_block', # Topic name
        Block, # Message type
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
        self.blockList = []
        heapq.heapify()

    def detection_callback(fiducialTransformArray: FiducialTransformArray):
        self.transformList = fiducialTransformArray.transforms


    def calibrate(self, Transf: Transform):
        rotQ = Transf.rotation
        transQ = Transf.translation
        rotM = R.as_matrix(R.from_quat([rotQ.x,rotQ.y,rotQ.z,rotQ.w]))
        transM = np.arry([transQ.x, transQ.y, transQ.z])*1000
        Tx = mr.RpToTrans(rotM, transM)
        self.Trx = mr.TransInv(Tx*mr.TransInv(self.Tr))
        self.calibrated = True
        
    def initialCalibration(self):
        if self.transformsList == None:
            return "No aruco cubes detected"
        for fiducial in self.transformsList:
            if(fiducial.fiducial_id == calibration_ID):
                self.calibrate(fiducial.transform)
                return ("successfully calibrated. Trx =", self.Trx)
        return "calibration ID not found"


        


   



    


if __name__ == '__main__':

    rospy.init_node('detect_block')
    #set frequency to increase performance
    rate = rospy.Rate(10)
    detectBlock = DetectBlock()
    while not detectBlock.calibrated:
        detectBlock.initialCalibration()
    while not rospy.is_shutdown():
        #publish message
        rate.sleep()


