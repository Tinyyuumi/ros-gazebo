#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/laneline_info话题，自定义消息类型laneline_publisher::laneline

import rospy
import tf
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
rospy.init_node('laneline_subscriber', anonymous=True)
listener = tf.TransformListener()


def getYaw(pose):
    q = tf.transformations.euler_from_quaternion((pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))
    return q[2]


def lanelineInfoCallback(msg):
    # pathpose = []
    odom = rospy.wait_for_message("/odom", Odometry)
    # for i in range(len(msg.poses)):
    new  = listener.transformPose("odom",msg.poses[0])
    #     pathpose.append(getYaw(new.pose))
    # carpose = getYaw(odom.pose.pose)
    pose = getYaw(new.pose)
    print(pose)
    vv = 1
    
    
        
        

def laneline_subscriber():
	# ROS节点初始化
    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, lanelineInfoCallback)
    rospy.Subscriber("/odom", Odometry)
    rospy.spin()


if __name__ == '__main__':
    laneline_subscriber()
