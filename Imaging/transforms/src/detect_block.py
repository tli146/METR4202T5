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

calibration_ID = 13
ros_rate = 2
rotation_theta_threshold = 0.5

class DetectedBlock:

    def __init__(self, id, Transf, Toc) -> None:
        rotQ = Transf.rotation
        transQ = Transf.translation
        rotM = R.from_quat([rotQ.x,rotQ.y,rotQ.z,rotQ.w] )
        rotM = R.as_matrix(rotM)
        transM = np.array([transQ.x, transQ.y, transQ.z])
        Tca = mr.RpToTrans(rotM, transM)


        Toa = np.dot(Toc,Tca)

        r, p = mr.TransToRp(Toa)

        self.id = id
        self.coordinate = p

        theta_1 = np.arctan2(r[0][1], r[0][2])
        theta_1 = theta_1%(np.pi/4)
        theta_2 = np.arctan2(p[1], p[0])
        self.theta = np.abs(theta_1 - theta_2)
        self.absTheta = theta_1
        self.priority = 0




    def setColor(self, color:int):
        self.color = color
    
    def setPriority(self, priority: int):
        if self.id == calibration_ID:
            return False
        self.priority = priority 

    def toMsg(self) -> Block:
        msg = Block()
        Block.id = self.id
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
        'block_detect',
        String
        ) 

        self.sub_fiducials_transform = rospy.Subscriber(
        "fiducial_transforms",
        FiducialTransformArray,
        self.detection_callback       
        )
        self.calibrated = False

        #set calibration aruco code location
        self.Tox = np.array([
            [1,0,0,0],
            [0,-1,0,-0.19],
            [0,0,-1,0],
            [0,0,0,1]
        ])

        self.Toc = []

        self.transformList = []
        self.blockList = []
        self.rotating = False



    def find_transM(self, Transf:Transform):
        rotQ = Transf.rotation
        transQ = Transf.translation
        rotM = R.from_quat([rotQ.x,rotQ.y,rotQ.z,rotQ.w] )
        rotM = R.as_matrix(rotM)
        transM = np.array([transQ.x, transQ.y, transQ.z])
        return mr.RpToTrans(rotM, transM)



    def calibrate(self, Transf: Transform):
        Tcx = self.find_transM(Transf)
        self.Toc = np.dot(self.Tox, mr.TransInv(Tcx))
        self.calibrated = True
        
        

        
    def initialCalibration(self):
        listID = []
        if self.transformList == None:
            return "No aruco cubes detected"
        for fiducial in self.transformList:
            if(fiducial.fiducial_id == calibration_ID):
                    self.calibrate(fiducial.transform)
                    return str(self.Toc)
            listID.append(fiducial.fiducial_id)
        return "calibration id not found" 




    def track_fiducial(self, id):
        for fiducial in self.transformList:
            if(fiducial.fiducial_id == id):
                block = DetectedBlock(id, fiducial.transform, self.Toc)


                detectBlock.pubCalibration.publish(str(block.coordinate))    
            



    def findPriorityBlock(self):
        #require system to be calibrated

        newBlocks = []
        for fiducial in self.transformList:
            newBlock = DetectedBlock(fiducial.fiducial_id, fiducial.transform, self.Toc)
            newBlocks.append(newBlock)
        #add blocks

        self.rotating = self.rotationDetect(newBlocks, self.blockList)
        #check for rotation with theta
        self.blockList = newBlocks
        #updating blocklist


        pubEmpty = False
        numBlocks = len(self.blockList)
        if numBlocks ==0:
            pubEmpty = True
            
        if numBlocks == 1 and self.blockList[0].id == calibration_ID:
            pubEmpty = True
            
        if self.rotating:
            pubEmpty = True

        if pubEmpty:
            emptyBlock = Block()
            emptyBlock.wait = True
            self.pub.publish(emptyBlock)

        
        #publish empty wait if no tags detected or only calibration is detected

        for i in self.blockList:
            sumDis = 0
            for j in self.blockList:

                #find distance to other blocks
                deltaX = i.coordinate[0] - j.coordinate[0]
                deltaY = i.coordinate[1] - j.coordinate[1]
                dist = np.sqrt(deltaX^2 + deltaY^2)
                if(dist > 50):
                    dist = 100
                if(dist < 20):
                    dist = 0
                sumDis += dist
            distWeight = sumDis/numBlocks
            priority = i.theta/np.pi*180 + distWeight + i.coordinate[1]/ 2
            i.setPriority(priority)

            #find highest priority (lower better)
        currentBlock = block[0]
        if currentBlock.id == calibration_ID:
            currentBlock = block[1]
        for block in self.blockList:
            if not block.id == calibration_ID:
                if block < currentBlock:
                    currentBlock = block
            
        blockMsg = currentBlock.toMsg()
        blockMsg.wait = False
        self.pub.publish(blockMsg)
        
        



            
                
       

    def rotationDetect(self, newBlocks, blockList):
        for i in newBlocks:
            for j in blockList:
                if i.id == j.id:
                    if np.abs(i.absTheta - j.absTheta) > rotation_theta_threshold:
                        return True
                    return False
                
                        


            



if __name__ == '__main__':

    rospy.init_node('detect_block')
    #set frequency to increase performance
    rate = rospy.Rate(ros_rate)
    detectBlock = DetectBlock()
    detectBlock.pubCalibration.publish("Starting calibration")
    #while not detectBlock.calibrated:
    
        
    
    while not rospy.is_shutdown():
        
        if not detectBlock.calibrated:
            detectBlock.initialCalibration()
            
        else:
           detectBlock.track_fiducial(calibration_ID)
        rate.sleep()


