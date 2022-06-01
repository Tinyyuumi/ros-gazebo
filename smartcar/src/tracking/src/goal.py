#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/laneline_info话题，自定义消息类型laneline_publisher::laneline

import rospy
import tf,math
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import Int8
from sensor_msgs.msg import LaserScan
import numpy as np 
import time
class Goalrun():
    def __init__(self):
        rospy.init_node('Goal', anonymous=True)
        self.listener = tf.TransformListener()
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.pathCallback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback,queue_size=10)
        self.pub_vel = rospy.Publisher("/con_cmd_vel",Twist,queue_size=1)
        self.pub_new_goal = rospy.Publisher("/robot/goal",PoseStamped,queue_size=1)
        rospy.Subscriber("/robot/arrive",Int8,self.backCallback)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)


        # params
        self.path = Path()
        self.odom = Odometry()
        self.goal = PoseStamped()
        self.goal_raw = PoseStamped()
        self.vel = Twist()

    def backCallback(self,data):
        if data.data == 1:
            goalYaw = self.getYaw(self.goal.pose)
            self.PID_spin(goalYaw)
            print("Goal Reached !")
        # 等小车到达位置

    
    def pathCallback(self,data):
        self.path = data

    def odomCallback(self,data):
        self.odom = data
        # carYaw = self.getYaw(self.odom.pose.pose)
        # print(carYaw)
    
    def goalCallback(self,data):
        self.goal_raw = data
        self.path = Path()
        while len(self.path.poses) <= 6:
            pass

        self.listener.waitForTransform("odom", "map", rospy.Time.now(), rospy.Duration(4.0))
        self.goal = self.listener.transformPose("odom",data)
        pathodom = self.listener.transformPose("odom",self.path.poses[6])
        
        pathyaw = math.atan2(pathodom.pose.position.y - self.odom.pose.pose.position.y,pathodom.pose.position.x - self.odom.pose.pose.position.x)
        
        self.PID_spin(pathyaw)
        self.pub_new_goal.publish(self.goal)
        # a = rospy.wait_for_message("/robot/arrive",Int8)
        # if a.data == 1:
        #     goalYaw = self.getYaw(self.goal.pose)
        #     self.PID_spin(goalYaw)
        #     print("Goal Reached !")
        # 等小车到达位置
        vv = 1


    def PID_spin(self,targetYaw):
        K = [5,0.5]  # pid
        error = [0,0]

        while(1):
            carYaw = self.getYaw(self.odom.pose.pose)
            error[0] = targetYaw - carYaw

            if abs(error[0]) < 0.3:
                self.vel.angular.z = 0
                self.pub_vel.publish(self.vel)
                break

            self.vel.angular.z = K[0]*error[0] + K[1]*error[1]
            if abs(self.vel.angular.z) > 1.0:
                self.vel.angular.z = 1.0*self.vel.angular.z/abs(self.vel.angular.z)
            error[1] += error[0]
            self.pub_vel.publish(self.vel)




    def getYaw(self,pose):
        q = tf.transformations.euler_from_quaternion((pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))
        return q[2]
    
    def angle_right(self,angle):
        angle1 = angle - 2*math.pi*np.floor((angle+math.pi)/(2*math.pi))
        return angle1



if __name__ == '__main__':
    goal = Goalrun()
    rospy.spin()

