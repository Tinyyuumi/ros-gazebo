#!/usr/bin/env python
# coding:utf-8

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import sys,copy
import datetime,time
import search
import obstacle
import threading

class Path():
    def __init__(self,begin=(90,5),end=(10,90)): # map
        # 速度和角速度
        self.msg1 = Twist()
        self.msg1.linear.x = 0.3
        self.msg1.linear.y = 0
        self.msg1.linear.z = 0
        self.msg1.angular.x = 0
        self.msg1.angular.y = 0
        self.msg1.angular.z = 0

        # 小车参数
        self.car_r = 0.2 # 小车半径

        # 导航使用的变量
        self.begin = begin # 起点
        self.end = end   # 终点

        self.num = 0        # 下一个目标点
        self.scale = 0.1    # 地图比例尺
        self.theta = 0      # 前一个点到下一个点向量的角度
        self.K_dis = [0.5,0.01,0.5] # Kp Ki Kd pid控制参数
        self.dis_err = [0,0,0] # 距离的本次偏差，上次偏差，累计偏差
        self.theta_err = 0     # 角度的本次偏差
        
        
        self.line = [0,0,0,0] # 直线方程，a,b,c,根号a方+b方
        self.dis_num = int(0)  
        self.flag = 0          

        self.pose = [self.scale*self.begin[0],self.scale*self.begin[1],1.57,0,0] # x y theta v omiga 姿态

        # 路径规划类
        self.path_plan = search.PathPlanning(self.begin,self.end)
        self.keypath = self.path_plan.path() # ///

        # 图像类
        self.view = obstacle.Vedio()

        # 障碍物距离,判断是否停下
        self.mindis = 0

        # 摄像头角度
        self.camera_theta = Float64()
        self.camera_theta.data = 0.0

    
    def reset(self):
        self.num = 0        # 下一个目标点
        self.theta = 0      # 前一个点到下一个点向量的角度
        self.dis_err = [0,0,0] # 距离的本次偏差，上次偏差，累计偏差
        self.theta_err = 0     # 角度的本次偏差

        self.dis_num = int(0)  
        self.flag = 0  
        

    def evolve1(self):
        while(1):
            # 获取位姿
            msg = rospy.wait_for_message("/robot/odom", Odometry)

            z = math.atan2(2*(msg.pose.pose.orientation.z*msg.pose.pose.orientation.w + msg.pose.pose.orientation.y*msg.pose.pose.orientation.x), 
                                1 - 2*(msg.pose.pose.orientation.y**2 + msg.pose.pose.orientation.z**2))

            if self.mindis == 1:
                self.msg1.linear.x = 0
                self.msg1.angular.z = 0
                vic_pub.publish(self.msg1)
                continue

            if self.num == len(self.keypath):
                break

            if z < 0:
                z = z + 6.28

            self.pose[0] = msg.pose.pose.position.x
            self.pose[1] = msg.pose.pose.position.y
            self.pose[2] = z
            self.pose[3] = msg.twist.twist.linear.x
            self.pose[4] = msg.twist.twist.angular.z

            # 等待转好
            if self.flag == 1:
                delta_theta = math.radians(self.theta) - self.pose[2]
                
                if abs(delta_theta) < 0.1:
                    self.flag = 0
                    continue

                self.msg1.angular.z = delta_theta
                self.msg1.linear.x = 0
                vic_pub.publish(self.msg1)
                continue
            

            # 判断是否达到目的地
            dis = math.sqrt((self.pose[0] - self.scale*self.keypath[self.num][0])**2 + (self.pose[1] - self.scale*self.keypath[self.num][1])**2)
            if dis < 0.1 and abs(self.num)<len(self.keypath):
                self.num = self.num + 1
                # goal = [keypath[num][0]*scale,keypath[num][1]*scale]
                if abs(self.num) == len(self.keypath):
                    print("mission finish")
                    self.msg1.linear.x = 0
                    self.msg1.angular.z = 0
                    vic_pub.publish(self.msg1)
                    continue

                derta = np.array(self.keypath[self.num])-np.array(self.keypath[self.num-1])

                # 偏差线
                # a = np.where(derta == 0) # 直线
                # self.dis_num = a[0][0]
                self.line = [self.scale*derta[1],-1*self.scale*derta[0],self.scale**2 *(self.keypath[self.num-1][1]*derta[0]-derta[1]*self.keypath[self.num-1][0]),0]
                self.line[3] = math.sqrt(self.line[0]**2 + self.line[1]**2) # 斜线
                
                # 计算转角
                self.theta = math.degrees(math.atan2(float(derta[1]),float(derta[0])))%360
                car_theta = math.degrees(self.pose[2])%360
                delta_theta = math.radians(self.theta - car_theta)

                # pid误差清零
                self.dis_err = [0,0,0]
                self.flag = 1
                self.msg1.angular.z = delta_theta
                self.msg1.linear.x = 0
                vic_pub.publish(self.msg1)

            # 直线的pid控制
            # self.dis_err[0] = self.pose[self.dis_num] - self.scale*self.keypath[self.num][self.dis_num]

            # 斜线的pid
            self.dis_err[0] = (self.line[0]*self.pose[0] + self.line[1]*self.pose[1] +self.line[2])/self.line[3]

            self.theta_err = math.radians(self.theta)- self.pose[2]
            
            self.msg1.angular.z = self.K_dis[0]*self.dis_err[0] + self.K_dis[1]*self.dis_err[2] + self.K_dis[2]*(self.dis_err[0]-self.dis_err[1])

            a = 2.0/(1+math.exp(-14*abs(self.dis_err[0]))) - 1

            if self.dis_err[0]*self.theta_err < 0:
                self.msg1.angular.z = a*self.msg1.angular.z + (1-a)*self.theta_err

            self.dis_err[2] += self.dis_err[0]
            self.dis_err[1] = self.dis_err[0]

            if self.msg1.angular.z > 0.5:
                self.msg1.angular.z = 0.5
            if self.dis_err[0] < 0.02:
                self.dis_err[2] = 0

            
            self.msg1.linear.x = 0.3
            # print(self.pose[0],self.pose[1],self.dis_err,self.theta_err)
            print(self.keypath[self.num])
            vic_pub.publish(self.msg1)


    def evolve2(self):
        while(1):
            # 获取图像
            image = rospy.wait_for_message("camera/image_raw",Image)
            d_,y_,(Mx,My) = self.view.image(image)

            if Mx != 0 and My != 0:
                # 转到合适角度                
                if Mx < 310:
                    self.camera_theta.data += 0.05
                    vedio_pub.publish(self.camera_theta)
                    time.sleep(0.2)
                elif Mx > 330:
                    self.camera_theta.data -= 0.05
                    vedio_pub.publish(self.camera_theta)
                    time.sleep(0.2)

                # 计算坐标
                if d_ < 1.5:
                    if self.camera_theta.data > 0:
                        d_ = d_ + self.view.r
                        y_ = y_ + self.view.r
                    else:
                        d_ = d_ + self.view.r
                        y_ = y_ - self.view.r

                    position_cartobox = [d_*math.cos(self.camera_theta.data)-y_*math.sin(self.camera_theta.data),d_*math.sin(self.camera_theta.data)+y_*math.cos(self.camera_theta.data)]
                    
                    a1 = self.pose[0] + (position_cartobox[0]+self.car_r)*math.cos(self.pose[2])-(position_cartobox[1]+self.car_r)*math.sin(self.pose[2])
                    a2 = self.pose[1] + (position_cartobox[0]+self.car_r)*math.sin(self.pose[2])+(position_cartobox[1]+self.car_r)*math.cos(self.pose[2])
                    position_worldtobox = [a1,a2]
                    
                    pos_inmap = [int(a1/self.scale),int(a2/self.scale)]
                    print(pos_inmap)

                    if self.path_plan.Map[pos_inmap[0]][pos_inmap[1]] == 1:
                        self.mindis = 0
                    
                    else:
                        self.mindis = 1
                        self.path_plan.Map = np.array(self.path_plan.Mapcopy)
                        self.path_plan.Map[pos_inmap[0]-3:pos_inmap[0]+4,pos_inmap[1]-3:pos_inmap[1]+4] = np.ones([7,7])
                        self.path_plan.bigger(5)

                        # t = self.path_plan.dirs[0]
                        # self.path_plan.dirs[0] =  self.path_plan.dirs[1]
                        # self.path_plan.dirs[1] = t

                        self.path_plan.start = (int(self.pose[0]/self.scale),int(self.pose[1]/self.scale))
                        self.keypath = self.path_plan.path()
                        self.reset()
                        self.mindis = 0

                        self.camera_theta.data = 0
                        vedio_pub.publish(self.camera_theta)
                        time.sleep(10)
                    continue


            self.camera_theta.data = 0
            vedio_pub.publish(self.camera_theta)
            self.mindis = 0

            
                 



rospy.init_node('getodom')
rate = rospy.Rate(50)
image_sub = rospy.Subscriber('camera/image_raw', Image)
vic_pub = rospy.Publisher('/robot/cmd_vel',Twist,queue_size=1)
vedio_pub = rospy.Publisher('/robot/yun_joint_position_controller/command',Float64,queue_size=1)

slam = Path()

thread1 = threading.Thread(target=slam.evolve1)
thread2 = threading.Thread(target=slam.evolve2)

thread2.start()
thread1.start()

rospy.spin()


vv = 1