#!/usr/bin/python
# -*- coding: UTF-8 -*- 
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import message_filters

amcl = open("/home/ab/amcl.txt", "w")
pose = open("/home/ab/pose.txt", "w")
real = open("/home/ab/real.txt", "w")
i = 0
# def amclcallback(data):
#     msg2 = rospy.wait_for_message("/odom", Odometry)

#     q=euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))
#     global amcl,real_am
#     amcl.write("%0.4f,%0.4f,%0.4f\n" % (data.pose.pose.position.x,data.pose.pose.position.y,q[2]))

#     q=euler_from_quaternion((msg2.pose.pose.orientation.x,msg2.pose.pose.orientation.y,msg2.pose.pose.orientation.z,msg2.pose.pose.orientation.w))
#     real_am.write("%0.4f,%0.4f,%0.4f\n" % (msg2.pose.pose.position.x,msg2.pose.pose.position.y,q[2]))

# def posecallback(data):
#     msg2 = rospy.wait_for_message("/odom", Odometry)
#     # msg1 = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)

#     q=euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))
#     global pose,real_pose
#     pose.write("%0.4f,%0.4f,%0.4f\n" % (data.pose.pose.position.x,data.pose.pose.position.y,q[2]))

#     # # msg1 = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
#     # q=euler_from_quaternion((msg1.pose.pose.orientation.x,msg1.pose.pose.orientation.y,msg1.pose.pose.orientation.z,msg1.pose.pose.orientation.w))
#     # amcl.write("%0.4f,%0.4f,%0.4f\n" % (msg1.pose.pose.position.x,msg1.pose.pose.position.y,q[2]))
#     # msg2 = rospy.wait_for_message("/odom", Odometry)
#     q=euler_from_quaternion((msg2.pose.pose.orientation.x,msg2.pose.pose.orientation.y,msg2.pose.pose.orientation.z,msg2.pose.pose.orientation.w))
#     real_pose.write("%0.4f,%0.4f,%0.4f\n" % (msg2.pose.pose.position.x,msg2.pose.pose.position.y,q[2]))


def multi_callback(subcriber_amcl,subcriber_pose,subcriber_real):
    global amcl,pose,real,i
    q1=euler_from_quaternion((subcriber_amcl.pose.pose.orientation.x,subcriber_amcl.pose.pose.orientation.y,subcriber_amcl.pose.pose.orientation.z,subcriber_amcl.pose.pose.orientation.w))
    q2=euler_from_quaternion((subcriber_pose.pose.pose.orientation.x,subcriber_pose.pose.pose.orientation.y,subcriber_pose.pose.pose.orientation.z,subcriber_pose.pose.pose.orientation.w))
    q3=euler_from_quaternion((subcriber_real.pose.pose.orientation.x,subcriber_real.pose.pose.orientation.y,subcriber_real.pose.pose.orientation.z,subcriber_real.pose.pose.orientation.w))
    
    amcl.write("%0.4f,%0.4f,%0.4f\n" % (subcriber_amcl.pose.pose.position.x,subcriber_amcl.pose.pose.position.y,q1[2]))
    pose.write("%0.4f,%0.4f,%0.4f\n" % (subcriber_pose.pose.pose.position.x,subcriber_pose.pose.pose.position.y,q2[2]))
    real.write("%0.4f,%0.4f,%0.4f\n" % (subcriber_real.pose.pose.position.x,subcriber_real.pose.pose.position.y,q3[2]))

    amcl.flush()
    pose.flush()
    real.flush()
    i = i + 1
    print(i)


def listener():
    global pose,real,amcl
    rospy.init_node('pose_listener', anonymous=True)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amclcallback)
    # rospy.Subscriber('/pose', PoseWithCovarianceStamped, posecallback)

    subcriber_amcl = message_filters.Subscriber('/amcl_pose', PoseWithCovarianceStamped, queue_size=1)
    subcriber_pose  = message_filters.Subscriber('pose', PoseWithCovarianceStamped, queue_size=1)
    subcriber_real  = message_filters.Subscriber('/odom', Odometry, queue_size=1)


    sync = message_filters.ApproximateTimeSynchronizer([subcriber_amcl,subcriber_pose,subcriber_real],10,0.1,allow_headerless=True)

    sync.registerCallback(multi_callback)

    # rospy.spin()

    # rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        pass
    amcl.close()
    pose.close()
    real.close()


if __name__ == '__main__':
    listener()

