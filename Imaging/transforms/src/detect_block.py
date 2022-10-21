#!/usr/bin/env python

from select import select
import rospy
import math
import heapq
from scipy.spatial.transform import Rotation as R
import numpy as np
import modern_robotics as mr

from metr4202_msgs.msg import block as Block
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import Transform
from std_msgs.msg import Header, String, Int16, Bool

calibration_ID = 11
ros_rate = 2

class DetectedBlock:
    def __init__(self, id, x,y,z,theta) -> None:
        self.id = id
        self.coordinate = (x,y,z)
        self.theta = theta
        self.priority = 0
        self.color = -1

    def setColor(self, color:int):
        self.color = color
    
    def setPriority(self, priority: int):
        self.priority = priority 

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
    def detection_callback(self, fiducialTransformArray: FiducialTransformArray):
        self.transformList = fiducialTransformArray.transforms


    def __init__(self):
        self.pub =rospy.Publisher(
        'priority_block', # Topic name
        Block, # Message type
        queue_size=10 # Topic size (optional)
        )

        self.pubCalibration = rospy.Publisher(
        'Calibration',
        String
        ) 

        self.sub_fiducials_transform = rospy.Subscriber(
        "fiducial_transforms",
        FiducialTransformArray,
        self.detection_callback       
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

        self.transformList = []
        self.blockList = []
        heapq.heapify(self.blockList)

    

    def calibrate(self, Transf: Transform):
        
        
        rotQ = Transf.rotation
        transQ = Transf.translation
        rotM = R.from_quat([rotQ.x,rotQ.y,rotQ.z,rotQ.w] )
        transM = np.array([transQ.x, transQ.y, transQ.z])
        Tx = mr.RpToTrans(rotM, transM)
        
        self.Trx = np.linalg.inv(Tx * np.linalg.inv(self.Tr))
        while(True):
            detectBlock.pubCalibration.publish(str(self.Trx))
        self.calibrated = True
        
        return True

        
    def initialCalibration(self):
        listID = []
        if self.transformList == None:
            return "No aruco cubes detected"
        for fiducial in self.transformList:
            if(fiducial.fiducial_id == calibration_ID):
                loop = True
                while(loop):
                    self.calibrate(fiducial.transform)
                    return str("successfully calibrated.")
            listID.append(fiducial.fiducial_id)
        return "calibration id not found" 


    def publishFiducials(self, stringPublisher):
        listID = []
        for fiducial in self.transformList:
             listID.append(fiducial.fiducial_id)
        listID.append(len(self.transformList))
        stringPublisher.publish(str(listID))


if __name__ == '__main__':

    rospy.init_node('detect_block')
    #set frequency to increase performance
    rate = rospy.Rate(ros_rate)
    detectBlock = DetectBlock()
    detectBlock.pubCalibration.publish("Starting calibration")
    #while not detectBlock.calibrated:
    
        
    
    while not rospy.is_shutdown():
        detectBlock.pubCalibration.publish(detectBlock.initialCalibration())
    
        #publish message
        rate.sleep()


