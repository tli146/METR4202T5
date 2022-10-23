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

calibration_ID = 2
ros_rate = 5
rotation_theta_threshold = 5 
#degrees

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
        self.coordinate = np.multiply(p, 1000)

        theta_1 = int(np.rad2deg(np.arctan2(r[0][0], r[0][1])))
        theta_1 = theta_1%90

        if(theta_1 >45):
            theta_1 = 90 - abs(theta_1)
        


        theta_2 = int(np.rad2deg(np.arctan2(p[1], p[0])))

        self.theta = np.abs(theta_1 - theta_2)
        self.absTheta = int(np.rad2deg(theta_1)%45)
        self.priority = 0




    def setColor(self, color:int):
        self.color = color
    
    def setPriority(self, priority: int):
        if self.id == calibration_ID:
            return False
        self.priority = priority 

    def toMsg(self) -> Block:
        msg = Block()
        msg.id = self.id
        msg.x = int(self.coordinate[0])
        msg.y = int(self.coordinate[1])
        msg.z = int(self.coordinate[2])
        msg.theta = self.theta
        
        return msg

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

    def _is_valid_operation(self, other):
        return type(self) == type(other)



class DetectBlock:
    def detection_callback(self, fiducialTransformArray: FiducialTransformArray):
        self.transformList = fiducialTransformArray.transforms
    
    
                
    def stateUpdater(self, data):
        self.state = data.data

    


    def __init__(self):
        self.pub =rospy.Publisher(
        'priority_block', # Topic name
        Block, # Message type
        )

        self.publish_block = rospy.Publisher(
        'block_detect',
        String
        )

        self.publish_raw = rospy.Publisher(
        'theta_raw',
        String
        )  

        self.publish_message = rospy.Publisher(
        'metr4202_message',
        String
        )  


        self.publish_state = rospy.Publisher(
        'metr4202_state',
        Int16
        )  

        self.sub_state = rospy.Subscriber(
        "metr4202_state",
        Int16,
        self.stateUpdater
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
            [0,-1,0,-0.190],
            [0,0,-1,0.015],
            [0,0,0,1]
        ])

        self.Toc = []
        self.state = 0
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
        self.publish_state.publish(1)
        
        

        
    def initialCalibration(self):
        listID = []
        if self.transformList == None:
            return "No aruco cubes detected"
        for fiducial in self.transformList:
            if(fiducial.fiducial_id == calibration_ID):
                    self.calibrate(fiducial.transform)
                    self.state = 1
                    self.publish_state.publish(self.state)
                    return str(self.Toc)
            listID.append(fiducial.fiducial_id)
        return "calibration id not found" 




    def track_fiducial(self, id):
        for fiducial in self.transformList:
            if(fiducial.fiducial_id == id):
                block = DetectedBlock(id, fiducial.transform, self.Toc)

                detectBlock.publish_block.publish(str(block.coordinate))    
                detectBlock.publish_raw.publish(str(block.absTheta))    
            



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
            self.publish_message.publish("no blocks")
            
        elif numBlocks == 1 and self.blockList[0].id == calibration_ID:
            pubEmpty = True
            self.publish_message.publish("only calibration cube found")
            
        elif self.rotating:
            pubEmpty = True
            #self.publish_message.publish("rotating")

        if pubEmpty:
            emptyBlock = Block()
            emptyBlock.wait = True
            self.pub.publish(emptyBlock)
            return

        
        #publish empty wait if no tags detected or only calibration is detected

        for i in self.blockList:
            sumDis = 0
            for j in self.blockList:

                #find distance to other blocks
                deltaX = i.coordinate[0] - j.coordinate[0]
                deltaY = i.coordinate[1] - j.coordinate[1]
                dist = np.sqrt( np.square(deltaX) + np.square(deltaY))
                if(dist > 70):
                    dist = 0
                if(dist < 65):
                    dist = 100
                
                sumDis += dist
            distWeight = sumDis/numBlocks

            priority = i.theta*3 + distWeight + np.abs(i.coordinate[1]/ 4)
            i.setPriority(priority)

            #find highest priority (lower better)

        currentBlock = self.blockList[0]
        if currentBlock.id == calibration_ID and len(self.blockList)>1:
            currentBlock = self.blockList[1]
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
                        rot = str(np.abs(i.absTheta - j.absTheta))
                        detectBlock.publish_message.publish(rot)
                        return True

                    return False
                
                        


            



if __name__ == '__main__':

    rospy.init_node('detect_block')
    #set frequency to increase performance
    rate = rospy.Rate(ros_rate)
    detectBlock = DetectBlock()
    
    #while not detectBlock.calibrated:
    
        
    
    while not rospy.is_shutdown():
        

        if not detectBlock.calibrated:

            if detectBlock.state == 10:
                detectBlock.initialCalibration()
                
            
        else:
            if detectBlock == 1:
                detectBlock.findPriorityBlock()
            # detectBlock.publish_message.publish("finding priority")
            

        rate.sleep()


