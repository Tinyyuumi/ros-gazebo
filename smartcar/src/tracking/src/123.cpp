/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include "control.hpp"

#define PI 3.14159265358979
int start_loop_flag = 0;

Controller::Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter
    pn.param("R", R, 0.2);  // car radius

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("baseSpeed", baseSpeed, 0.3);
    pn.param("goalRadius", goalRadius, 0.05);
    pn.param("forward_dis", forward_dis, 2);  //前馈点

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odom", 1, &Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base/GlobalPlanner/plan", 1, &Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &Controller::goalCB, this);

    pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &Controller::goalReachingCB, this); // Duration(0.05) -> 40Hz

    //Init variables
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

    car_stop = 0;
}


// Three callback
void Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}


void Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
    // controlLoopCB();
}


void Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0),*goalMsg, "map" ,odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;

    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}


//////
double Controller::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}


double Controller::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}


void Controller::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if(car2goal_dist < goalRadius)
        {
            goal_reached = true;
            goal_received = false;
            //ROS_INFO("Goal Reached !");
            car_stop = 100;
        }
    }
}

double Controller::getfirstpathyaw(const geometry_msgs::Pose& carPose,double yaw)
{

    geometry_msgs::PoseStamped odom_1_path;
    geometry_msgs::PoseStamped map_1_path = map_path.poses[0];
    tf_listener.transformPose("odom", ros::Time(0) , map_1_path, "map" ,odom_1_path);
    ROS_INFO("odomposition:%f,%f",odom_1_path.pose.position.x,odom_1_path.pose.position.y);
    geometry_msgs::Point odom_car2WayPtVec;
    odom_car2WayPtVec.x = cos(yaw)*(odom_1_path.pose.position.x - carPose.position.x) + sin(yaw)*(odom_1_path.pose.position.y - carPose.position.y);
    odom_car2WayPtVec.y = -sin(yaw)*(odom_1_path.pose.position.x - carPose.position.x) + cos(yaw)*(odom_1_path.pose.position.y - carPose.position.y);
    double first_angle = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);

    return first_angle;
}


//two judging
bool Controller::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}


bool Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < goalRadius)
        return false;
    else if(dist >= goalRadius)
        return true;
}

/// two important function
geometry_msgs::Point Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point odom_car2WayPtVec;
    geometry_msgs::Point forwardPt;
    foundForwardPt = false;

    if(!goal_reached){
        //路径平滑
        //放到一个节点去

        // 计算角速度
        int start_i = 0;
        if(map_path.poses.size()>forward_dis-1) start_i = forward_dis-1;

        for(int i = start_i; i< map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);

                if(_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        ROS_INFO("55555");
                        break;
                    }
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

        
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
    }

    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}


double Controller::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);

    double mk = 2*sin(eta)/(sqrt(odom_car2WayPtVec.y*odom_car2WayPtVec.y+odom_car2WayPtVec.x*odom_car2WayPtVec.x));
    return mk;
}



void Controller::controlLoopCB(const ros::TimerEvent&)
// void Controller::controlLoopCB()
{
    geometry_msgs::Pose carPose = odom.pose.pose;
    geometry_msgs::Twist carVel = odom.twist.twist;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    if(goal_received)
    {
        double eta = getEta(carPose); 
        ROS_INFO("11111:%f",eta);
        if(foundForwardPt)
        {
            // cmd_vel.angular.z = eta*carVel.x;
            if(!goal_reached)
            {
                if(cmd_vel.linear.x > baseSpeed)   cmd_vel.linear.x = baseSpeed;
            }
            else
            {
                cmd_vel.linear.x = 0;
            }
            ROS_INFO("22222:%f",baseSpeed);
            ROS_INFO("33333:%f",cmd_vel.linear.x);
            cmd_vel.angular.z = eta*cmd_vel.linear.x;
        }


    }
    if(car_stop > 0)
    {
        start_loop_flag = 0;
        if(carVel.linear.x > 0)
        {

            cmd_vel.linear.x = 0.0; //反向刹车
            pub_.publish(cmd_vel);
        }
        else
        {
            car_stop = 0;
            cmd_vel.linear.x = 0.0;
            pub_.publish(cmd_vel);

        }
    }
    else
    {
        pub_.publish(cmd_vel);
        car_stop = 0;
    }
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "car_controller");
    Controller controller;
    
    ros::spin();
    return 0;
}
