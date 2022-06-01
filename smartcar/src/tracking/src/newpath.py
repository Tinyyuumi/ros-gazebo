#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/laneline_info话题，自定义消息类型laneline_publisher::laneline

import rospy
import tf,copy,math
from nav_msgs.msg import Path,Odometry,OccupancyGrid
from geometry_msgs.msg import PoseStamped,Point,Twist,PoseWithCovarianceStamped
import numpy as np

class Tracking():
    def __init__(self):
        rospy.init_node('Tracking', anonymous=True)

        # car params
        controller_freq = 20
        self.goalRadius = 0.1 # 到达目标点容忍距离
        self.baseSpeed = 0.4 # 机器人速度
        self.Lfw = 0.4 # 最小前馈距离

        # subscriber
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback,queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.costmapCallback)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.pathCallback)

        # rospy.Subscriber("/robot/goal", PoseStamped, self.goalCallback)

        # publisher
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_pid = rospy.Publisher("/con_cmd_vel",Twist,queue_size=1)
        self.pub_pure = rospy.Publisher("/nav_cmd_vel",Twist,queue_size=1)


        # timer
        self.timer1 = rospy.Timer(rospy.Duration((1.0)/controller_freq),self.time1callback) # pid timer, check arrival timer
        self.timer2 = rospy.Timer(rospy.Duration((1.0)/controller_freq),self.time2callback) # tracking timer

        # tf listener
        self.listener = tf.TransformListener()


        # path params
        self.odom = Odometry()
        self.goal = PoseStamped()
        self.allpath = Path()
        self.currentgoal = PoseStamped()
        self.costmap = OccupancyGrid()
        self.path = Path()
        

        # symbol params
        self.symbol = [0,0,0,0,0,0] # start pid, start tracking, check arrive current pose, check final pose,end pid,  end tracking
        self.foundForwardPt = False # path symbol

        # other params
        self.vel_pid = Twist()
        self.cmd_vel = Twist()

        self.path_size = [0,1,0] # allpath size, pose pathsize, +20*x
        
    def pathCallback(self,data):
        if self.symbol[1] == 1:
            self.path = data

    def goalCallback(self,data):
        if self.symbol[5] == 0:
            self.goal = data
            self.allpath = rospy.wait_for_message("/move_base/GlobalPlanner/plan",Path)
            self.path_size[0] = len(self.allpath.poses)
            self.symbol[0] = 1
            self.symbol[5] = 1
            self.restart()


    def odomCallback(self,data):
        self.odom = data
    
    def costmapCallback(self,data):
        self.costmap = data
        vv = 1

    def restart(self):
        self.baseSpeed = 0.4
        self.path_size = [0,1,0]

    
    def time1callback(self,event):
        if self.symbol[0] == 1:
            self.listener.waitForTransform("odom", "map", rospy.Time(0), rospy.Duration(4.0))
            path_odom = self.listener.transformPose("odom",self.allpath.poses[6])
            pathyaw = math.atan2(path_odom.pose.position.y - self.odom.pose.pose.position.y,path_odom.pose.position.x - self.odom.pose.pose.position.x)
            self.PID_spin(pathyaw)
            self.symbol[0] = 0
            self.symbol[1] = 1
        
        if self.symbol[1] == 1:
            if self.symbol[2] == 0 and self.symbol[3] == 0:
                if (self.path_size[0] > 100*self.path_size[1]+self.path_size[2]):
                    occur = self.worldstomap_occur(self.allpath.poses[100*self.path_size[1]+self.path_size[2]].pose.position.x,self.allpath.poses[100*self.path_size[1]+self.path_size[2]].pose.position.y)
                    if occur < 80:
                        self.pub_goal.publish(self.allpath.poses[100*self.path_size[1]+self.path_size[2]])
                        self.path_size[2] = 0
                        self.listener.waitForTransform("odom", "map",rospy.Time(0), rospy.Duration(4.0))
                        self.currentgoal = self.listener.transformPose("odom",self.allpath.poses[100*self.path_size[1]+self.path_size[2]])
                        self.symbol[2] = 1

                    else:
                        self.path_size[2] += 20
                        return 0
                else:
                    self.pub_goal.publish(self.goal)
                    self.listener.waitForTransform("odom", "map", rospy.Time(0), rospy.Duration(4.0))
                    self.currentgoal = self.listener.transformPose("odom",self.goal)
                    self.symbol[3] = 1

            elif self.symbol[2] == 1:
                car2goal_dis = self.getCar2GoalDist(self.currentgoal)
                if(car2goal_dis<0.3):
                    self.symbol[2] = 0

            elif self.symbol[3] == 1:
                car2goal_dis = self.getCar2GoalDist(self.currentgoal)
                if(car2goal_dis<self.goalRadius):
                    self.symbol[3] = 0
                    self.symbol[4] = 1
                    self.symbol[1] = 0
            else:
                pass
            
        if self.symbol[4] == 1:
            # self.listener.waitForTransform("odom", "map", rospy.Time.now(), rospy.Duration(4.0))
            # path_odom = self.listener.transformPose("odom",self.currentgoal)
            goalYaw = self.getYaw(self.currentgoal.pose)
            self.PID_spin(goalYaw)
            self.symbol = [0,0,0,0,0,0]



    def getCar2GoalDist(self,goal):
        current_odom = copy.deepcopy(self.odom.pose.pose.position)
        car2goal_x = current_odom.x - goal.pose.position.x
        car2goal_y = current_odom.y - goal.pose.position.y
        dist2goal = math.sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y)
        return dist2goal
            

    def PID_spin(self,targetYaw):
        K = [5,0.5]  # pid
        error = [0,0]

        while(1):
            carYaw = self.getYaw(self.odom.pose.pose)
            error[0] = targetYaw - carYaw

            if abs(error[0]) < 0.2:
                self.vel_pid.angular.z = 0
                self.pub_pid.publish(self.vel_pid)
                break

            self.vel_pid.angular.z = K[0]*error[0] + K[1]*error[1]
            if abs(self.vel_pid.angular.z) > 1.0:
                self.vel_pid.angular.z = np.sign(self.vel_pid.angular.z)
            error[1] += error[0]
            self.pub_pid.publish(self.vel_pid)

    def getYaw(self,pose):
        q = tf.transformations.euler_from_quaternion((pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))
        return q[2]

    def worldstomap_occur(self,x_world,y_world):
        x_map = (math.floor((x_world - self.costmap.info.origin.position.x) / self.costmap.info.resolution + 0.5) + self.costmap.info.width / 2)
        y_map = (math.floor((y_world - self.costmap.info.origin.position.y) / self.costmap.info.resolution + 0.5) + self.costmap.info.height / 2)
        occur = self.costmap.data[x_map + y_map * self.costmap.info.width]
        return occur

    ''' timer 2 '''
    def time2callback(self,event):
        q = copy.deepcopy(self.odom)
        carPose = q.pose.pose
        carVel = q.twist.twist
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        print(self.symbol)
        if(self.symbol[1]==1):
            eta = self.getEta(carPose)
            if (self.foundForwardPt):
                # self.vel_adjust()
                self.cmd_vel.linear.x = self.baseSpeed
                self.cmd_vel.angular.z = eta*self.cmd_vel.linear.x
            self.pub_pure.publish(self.cmd_vel)
            print("bbb")


    def getEta(self,carPose):
        odom_car2WayPtVec = self.get_odom_car2WayPtVec(carPose)
        eta = math.atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x)
        d = math.sqrt(odom_car2WayPtVec.y*odom_car2WayPtVec.y+odom_car2WayPtVec.x*odom_car2WayPtVec.x)

        # d,eta = self.get_odom_car2WayPtVec(carPose)

        # self.baseSpeed = 0.3
        # self.baseSpeed = max(self.K * abs(eta) + self.maxSpeed,self.minSpeed)
        # print([eta,self.baseSpeed])
        mk = 2*math.sin(eta)/d
        return mk

    def get_odom_car2WayPtVec(self,carPose):
        carPose_pos = Point()
        carPose_pos = carPose.position
        carPose_yaw = self.getYaw(carPose)
        odom_car2WayPtVec = Point()
        myforwardPoint = Point()
        forwardPt = Point()
        # path_yaw = 0
        self.foundForwardPt = False
        if self.symbol[4]==0:
            path = copy.deepcopy(self.path.poses)
            if len(path)<20:
                dis = self.getCar2GoalDist(self.currentgoal)
                if dis < self.Lfw:
                    self.foundForwardPt = True
                    forwardPt = self.currentgoal.pose.position
                    self.baseSpeed = 0.1

            
            for i in range(len(path)):
                map_path_pose = path[i]
                # odom_path_pose = PoseStamped()
                # self.listener.waitForTransform("odom", "map", rospy.Time(), rospy.Duration(4.0))
                # odom_path_pose  = self.listener.transformPose("odom",map_path_pose)
                try:
                    self.listener.waitForTransform("odom", "map", rospy.Time(0), rospy.Duration(4.0))
                    odom_path_pose  = self.listener.transformPose("odom",map_path_pose)
                    odom_path_wayPt = odom_path_pose.pose.position
                    _isForwardWayPt = self.isForwardWayPt(odom_path_wayPt,carPose)
                    if(_isForwardWayPt):
                        _isWayPtAwayFromLfwDist = self.isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos)
                        if(_isWayPtAwayFromLfwDist):
                            forwardPt = odom_path_wayPt
                            # path_yaw = self.getYaw(odom_path_pose.pose)
                            self.foundForwardPt = True

                            # draw the point
                            # myforwardPoint.x = path[30].pose.position.x
                            # myforwardPoint.y = path[30].pose.position.y
                            # myforwardPoint.z = path[30].pose.position.z
                            # self.pub_point.publish(myforwardPoint)
                            break
                except:
                    pass

        elif(self.symbol[4]==1):
            forwardPt = self.odom.pose.pose.position
            self.foundForwardPt = False

            
        odom_car2WayPtVec.x = math.cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + math.sin(carPose_yaw)*(forwardPt.y - carPose_pos.y)
        odom_car2WayPtVec.y = -math.sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + math.cos(carPose_yaw)*(forwardPt.y - carPose_pos.y)

        # d = math.sqrt(odom_car2WayPtVec.y*odom_car2WayPtVec.y+odom_car2WayPtVec.x*odom_car2WayPtVec.x)
        # diff_angle = path_yaw - carPose_yaw

        return odom_car2WayPtVec


    def isForwardWayPt(self,wayPt,carPose):
        car2wayPt_x = wayPt.x - carPose.position.x
        car2wayPt_y = wayPt.y - carPose.position.y
        car_theta = self.getYaw(carPose)

        car_car2wayPt_x = math.cos(car_theta)*car2wayPt_x + math.sin(car_theta)*car2wayPt_y
        car_car2wayPt_y = -1*math.sin(car_theta)*car2wayPt_x + math.cos(car_theta)*car2wayPt_y

        if(car_car2wayPt_x >0):
            return True
        else:
            return False

    def isWayPtAwayFromLfwDist(self,wayPt,car_pos):
        dx = wayPt.x - car_pos.x
        dy = wayPt.y - car_pos.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < self.Lfw:
            return False
        else:
            return True

    
        
if __name__ == '__main__':
    track = Tracking()
    rospy.spin()
