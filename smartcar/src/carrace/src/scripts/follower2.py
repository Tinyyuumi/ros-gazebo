#!/usr/bin/env python
# coding:utf-8
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math

class Follower:
  def __init__(self):
    self.bridge1 = cv_bridge.CvBridge()
    self.bridge2 = cv_bridge.CvBridge()
    self.right_image_sub = rospy.Subscriber('/stereocamera/right/image_raw', 
                                    Image, self.right_image_callback,queue_size=1)

    self.left_image_sub = rospy.Subscriber('/stereocamera/left/image_raw', 
                                    Image, self.left_image_callback,queue_size=1)

    self.vic_pub = rospy.Publisher('/robot/cmd_vel',Twist,queue_size=1)

  def right_image_callback(self, msg):
      img1 = self.bridge1.imgmsg_to_cv2(msg)
      cv2.imshow('1',img1)
      cv2.waitKey(10)


  def left_image_callback(self, msg):
      img2 = self.bridge2.imgmsg_to_cv2(msg)
      print('a')
      # cv2.imshow('2',img2)
      # cv2.waitKey(10)

rospy.init_node('follower',anonymous=True)
follower = Follower()
rospy.spin()
