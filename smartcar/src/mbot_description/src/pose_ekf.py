#!/usr/bin/python
# -*- coding: UTF-8 -*- 
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import message_filters

lader = open("lider.txt", "w")
ekf = open("ekf.txt", "w")
real = open("real.txt", "w")
gps = open("gps.txt", "w")
imu = open("imu.txt", "w")
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


def multi_callback(subcriber_lader,subcriber_ekf,subcriber_odom,subcriber_gps,subcriber_imu):
    global lader,ekf,real,gps
    q1=euler_from_quaternion((subcriber_lader.pose.pose.orientation.x,subcriber_lader.pose.pose.orientation.y,subcriber_lader.pose.pose.orientation.z,subcriber_lader.pose.pose.orientation.w))
    q2=euler_from_quaternion((subcriber_ekf.pose.pose.orientation.x,subcriber_ekf.pose.pose.orientation.y,subcriber_ekf.pose.pose.orientation.z,subcriber_ekf.pose.pose.orientation.w))
    q3=euler_from_quaternion((subcriber_odom.pose.pose.orientation.x,subcriber_odom.pose.pose.orientation.y,subcriber_odom.pose.pose.orientation.z,subcriber_odom.pose.pose.orientation.w))
    q4=euler_from_quaternion((subcriber_imu.orientation.x,subcriber_imu.orientation.y,subcriber_imu.orientation.z,subcriber_imu.orientation.w))

    lader.write("%0.4f,%0.4f,%0.4f\n" % (subcriber_lader.pose.pose.position.x,subcriber_lader.pose.pose.position.y,q1[2]))
    ekf.write("%0.4f,%0.4f,%0.4f\n" % (subcriber_ekf.pose.pose.position.x,subcriber_ekf.pose.pose.position.y,q2[2]))
    real.write("%0.4f,%0.4f,%0.4f\n" % (subcriber_odom.pose.pose.position.x,subcriber_odom.pose.pose.position.y,q3[2]))
    gps.write("%0.4f,%0.4f\n" % (subcriber_gps.pose.pose.position.x,subcriber_gps.pose.pose.position.y))
    imu.write("%0.4f\n" % (q4[2]))

def listener():
    global pose,real,amcl
    rospy.init_node('pose_listener', anonymous=True)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amclcallback)
    # rospy.Subscriber('/pose', PoseWithCovarianceStamped, posecallback)

    subcriber_lader = message_filters.Subscriber('/odom_rf2o', Odometry, queue_size=1)
    subcriber_ekf  = message_filters.Subscriber('/odometry/filtered', Odometry, queue_size=1)
    subcriber_odom  = message_filters.Subscriber('/odom', Odometry, queue_size=1)
    subcriber_gps  = message_filters.Subscriber('/robot1/gps_sim', Odometry, queue_size=1)
    subcriber_imu  = message_filters.Subscriber('/imu_data', Imu, queue_size=1)

    sync = message_filters.ApproximateTimeSynchronizer([subcriber_lader,subcriber_ekf,subcriber_odom,subcriber_gps,subcriber_imu],10,0.1,allow_headerless=True)

    sync.registerCallback(multi_callback)

    # rospy.spin()

    # rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        pass
    lader.close()
    ekf.close()
    real.close()
    gps.close()
    imu.close()

if __name__ == '__main__':
    listener()

