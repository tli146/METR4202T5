#!/usr/bin/env python

import rospy
import cv2
import numpy as np 
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge, CvBridgeError


ros_rate = 10

class CameraViewer:

  def __init__(self, serial):
    self.bridge = CvBridge()
    self.serial = serial
    self.image_sub = rospy.Subscriber(f"/ximea_ros/ximea_{self.serial}/image_raw", Image, self.callback)
    self.color_pub = rospy.Publisher("/test_color", ColorRGBA, queue_size=10)
    self.img = None

  def callback(self,data):
    try:
      self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    

  def publish_color_img(self):
    img = self.img
    bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]
    color = ColorRGBA()
    color.r = bgr[2]
    color.g = bgr[1]
    color.b = bgr[0]
    self.color_pub.publish(color)





if __name__ == '__main__':  
  rospy.init_node('image_node', anonymous=True)
  viewer = CameraViewer('31706251')
  rate = rospy.Rate(ros_rate)
  try:
    while not rospy.is_shutdown():
        #if img is not None:
            #cv2.imshow("Image", img)
            #cv2.waitKey(1)
            #None
      viewer.publish_color_img()
      rate.sleep()
        

  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()