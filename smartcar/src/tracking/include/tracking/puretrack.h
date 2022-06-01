#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <vector>

using namespace std;

class Track
{
    public:
        void init();
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
        double getYawFromPose(const geometry_msgs::Pose& carPose);        
        double getEta(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        geometry_msgs::Twist PID(double);
    
    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub;
        ros::Publisher pub_;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;

        int flag, controller_freq;
        double baseSpeed, Lfw, R, goalRadius, realSpeed;
        bool foundForwardPt, goal_received, goal_reached;

        vector<double> k;
        vector<double> err;

        geometry_msgs::Twist cmd_vel;
        nav_msgs::Odometry odom;
        geometry_msgs::PoseStamped odom_goal;
        geometry_msgs::Point odom_goal_pos;
        nav_msgs::Path map_path, odom_path;
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void goalReachingCB(const ros::TimerEvent&);
        void controlLoopCB(const ros::TimerEvent&);
};