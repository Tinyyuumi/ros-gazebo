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

class Vedio():
    def __init__(self):
        # 距离
        Zc = 1.0

        # 相机参数
        fx = 381.35
        fy = 381.38
        u0 = 320
        v0 = 240

        # 内参矩阵
        self.A = np.array([[Zc/fx,0,-1*Zc*u0/fx],[0,Zc/fy,-1*Zc*v0/fy],[0,0,Zc],[0,0,1]])

        self.bridge = cv_bridge.CvBridge()
        self.color_lower = np.array([100, 43, 46])  # 蓝色下限
        self.color_upper = np.array([124, 255, 255]) # 蓝色上限

        # 障碍物尺寸
        self.h = 0.2 # 高度
        self.r = 0.3 # 宽度、半径
    def image(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg)

        # RGB图像转HSV图像
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  
        mask = cv2.inRange(img_hsv,self.color_lower,self.color_upper)
        img1 = cv2.GaussianBlur(mask,(5,5),0)

        try:
            # 框出障碍物
            ret, img_bin = cv2.threshold(img1, 127, 255, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_TREE , cv2.CHAIN_APPROX_SIMPLE )
            # cv2.drawContours(img,contours,-1,(255,0,255),3)
            num = len(contours)
            for i in range(num):
                area = cv2.contourArea(contours[i], oriented=False)
                if 100< area:  #限定轮廓的面积
                    rect = cv2.boundingRect(contours[i])

                    # 计算距离
                    p1 = np.array([rect[0],rect[1],1],dtype=float)
                    p2 = np.array([rect[0],rect[1]+rect[3],1],dtype=float)
                    # print(p1,p2)
                    p1_local = np.matmul(self.A,p1)
                    p2_local = np.matmul(self.A,p2)

                    Zc = self.h/abs(p2_local[1]-p1_local[1])
                    cv2.rectangle(img, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (255, 255, 255))

                    corners = cv2.goodFeaturesToTrack(img1,25,0.01,10)

            # 重心检测
            moment_blue = cv2.moments(img1)  
            Mx = int(moment_blue['m10']/moment_blue['m00'])
            My = int(moment_blue['m01']/moment_blue['m00'])
            cv2.circle(img,(Mx,My),2,(255,255,36),5) # 圆心，半径

            # 角点检测
            corners = cv2.goodFeaturesToTrack(img1,25,0.01,10)
            corners = np.int0(corners)
            list1 = corners.tolist()
            list1 = sorted(list1, key=(lambda x: [x[0][1],x[0][0]]))

            A = Zc * self.A
            p = np.array([list1[-1][0][0],list1[-1][0][1],1],dtype=float)
            p_local = np.matmul(A,p)

            # cv2.imshow('1',img)
            # cv2.waitKey(3)

            return Zc,p_local[0],(Mx,My)

        except:
            # cv2.imshow('1',img)
            # cv2.waitKey(3)
            return 100,100,(0,0)


    # rospy.init_node('avoid')
    # image_sub = rospy.Subscriber('camera/image_raw', Image, image_callback)
    # # vic_pub = rospy.Publisher('/robot/cmd_vel',Twist,queue_size=1)
    # rospy.spin()





