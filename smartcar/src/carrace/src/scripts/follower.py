#!/usr/bin/env python
# coding:utf-8
# BEGIN ALL
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math
import getcircle
import time

# 距离
Zc = 0.8

Z1 = 1 #320
Z2 = 1.5 #290
Z3 = 2 #275

Z11 = Z1/Zc
Z22 = Z2/Zc
Z33 = Z3/Zc

# 相机参数
fx = 381.35
fy = 381.38
u0 = 320
v0 = 240
A = np.array([[Zc/fx,0,-1*Zc*u0/fx],[0,Zc/fy,-1*Zc*v0/fy],[0,0,Zc],[0,0,1]])

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('camera/image_raw', 
                                    Image, self.image_callback)
    self.vic_pub = rospy.Publisher('/robot/cmd_vel',Twist,queue_size=1)

    self.msg = Twist()
    self.msg.linear.y = 0
    self.msg.linear.z = 0
    self.msg.angular.x = 0
    self.msg.angular.y = 0

    self.forward_angz = 0
    self.forward_linex = 0
    self.time = 0

    self.err = 0
    self.pr_err = 0
    self.Kp = 0.5
    self.Ki = 0.002
    self.Kd = 0.2

  def img_to_vic(self,edgess):
    # 取第370行作为距离参考
    line = edgess[370,:]
    point = np.where(line != 0)
    mid_line = np.array([np.mean(point),370,1],dtype=float)
    dis = np.matmul(A,mid_line)
    distance = abs(dis[0])

    if mid_line[0] >= 320:
      distance = -1*distance

    return distance

  def forward(self,edgess):
    x = []
    y = [1,1.7,2.5]
    line = edgess[320,:]
    point = np.where(line != 0)
    mid_line = np.array([np.mean(point),320,1],dtype=float)
    dis = np.matmul(Z11*A,mid_line)
    x.append(dis[0])

    line = edgess[300,:]
    point = np.where(line != 0)
    mid_line = np.array([np.mean(point),290,1],dtype=float)
    dis = np.matmul(Z22*A,mid_line)
    x.append(dis[0])

    line = edgess[280,:]
    point = np.where(line != 0)
    mid_line = np.array([np.mean(point),275,1],dtype=float)
    dis = np.matmul(Z33*A,mid_line)
    x.append(dis[0])

    # f = np.polyfit(x, y, 2)
    # f = np.where((abs(f)<0.01),0,f)
    # ddf = 2*f[0]
    print(x)

    self.time -= 1
    if self.time<0:
      self.time = 0

    if np.max(x) - np.min(x) < 0.1:
      self.forward_linex += 0.01
      if self.forward_linex > 0.3:
        self.forward_linex = 0.3
      return 0

    elif np.max(x) - np.min(x) > 0.4:
      a = getcircle.circle(x,y)
      if self.time == 0:
        self.time = 30*2*math.asin(a[0]/(2*a[1]))*a[1]/self.msg.linear.x
        # self.msg.angular.z = 2*math.asin(a[0]/(2*a[1]))/self.time
      print(self.time)
      return 0

    else:
      self.forward_linex -= 0.01
      if self.forward_linex < 0:
        self.forward_linex = 0
      return 0


  def PID(self,distance):
    self.err = math.atan(distance/Zc)
    self.msg.linear.x = 0.8 + self.forward_linex
    # if self.time == 0:
    self.msg.angular.z = self.Kp*self.err + self.Ki*(self.pr_err+self.err) + self.Kd*(self.err-self.pr_err)

    # self.msg.angular.z = self.err + self.forward_angz/self.msg.linear.x
    self.pr_err = self.err
    
    if distance == 0:
      self.err = 0
      self.pr_err = 0
    if self.msg.angular.z > 1.2:
      self.msg.angular.z = 1.2
    print(self.msg.linear.x)
    print(self.msg.angular.z)
    print('\n')
  def image_callback(self, msg):
    img = self.bridge.imgmsg_to_cv2(msg)
    img1 = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret,mask = cv2.threshold(img1,200,255,cv2.THRESH_BINARY)

    edges = cv2.Canny(mask,100,200)
    dis = self.img_to_vic(edges)
    self.forward(edges)
    self.PID(dis)
    # print time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(time.time()))
    # self.vic_pub.publish(self.msg)
    cv2.imshow('1',mask)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
