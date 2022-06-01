#!/usr/bin/python
# -*- coding: UTF-8 -*- 
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import message_filters

path = open("path.txt", "w")


def callback(data):
    global path
    q1=euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))
    path.write("%0.4f,%0.4f,%0.4f\n" % (data.pose.pose.position.x,data.pose.pose.position.y,q1[2]))



def listener():
    global path
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber('/odom', Odometry, callback)

    while not rospy.is_shutdown():
        pass
    path.close()

if __name__ == '__main__':
    listener()

