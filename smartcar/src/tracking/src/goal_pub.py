#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/laneline_info话题，自定义消息类型laneline_publisher::laneline

import rospy
import tf,copy,math
from nav_msgs.msg import Path,Odometry,OccupancyGrid
from geometry_msgs.msg import PoseStamped,Point,Twist,PoseWithCovarianceStamped
import numpy as np

class Goaling():
    def __init__(self):
        rospy.init_node('Goal', anonymous=True)

        # subscriber
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback,queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.pathCallback)

        # rospy.Subscriber("/robot/goal", PoseStamped, self.goalCallback)

        # publisher
        self.pub_goal = rospy.Publisher("/local/goal", PoseStamped, queue_size=1)

        self.timer1 = rospy.Timer(rospy.Duration((1.0)/20),self.time1callback) # pid timer, check arrival timer

        # path params
        self.goal = PoseStamped()
        self.allpath = Path()
    
    def goalCallback(self,data):
        self.goal = data

    def pathCallback(self,data):
        self.allpath = data
    
    def odomCallback(self,data):
        self.allpath = data

    def time1callback(self,data):



    
if __name__ == '__main__':
    track = Goaling()
    rospy.spin()
