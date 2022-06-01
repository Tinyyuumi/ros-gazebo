#include <tracking/puretrack.h>

void Track::init()
{
    //Private parameters handler
    ros::NodeHandle pn("~");
    
    //Car parameter
    pn.param("R", R, 0.2);  // 小车半径

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20); //控制频率
    pn.param("baseSpeed", baseSpeed, 0.4); //线速度
    pn.param("goalRadius", goalRadius, 0.1); // 目标点半径
    pn.param("forward_dis", Lfw, 0.2);  //前馈距离

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odom", 1, &Track::odomCB, this);
    path_sub = n_.subscribe("/move_base/GlobalPlanner/plan", 1, &Track::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &Track::goalCB, this);

    pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &Track::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &Track::goalReachingCB, this); // Duration(0.05) -> 40Hz

    //Init variables
    flag = 0;
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;

     k = {5,0.5,0,0};
     err = {0,0};

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    
    realSpeed = baseSpeed;

    
}

// Three callback
void Track::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}


void Track::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
    // controlLoopCB();
}


void Track::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        tf_listener.transformPose("odom", ros::Time(0),*goalMsg, "map" ,odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        flag = 1;
        goal_received = true;

    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

// 定时器1 回调
void Track::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if(car2goal_dist < goalRadius)
        {
            goal_reached = false;
            goal_received = true;
            realSpeed =baseSpeed;
            flag = 3;
            ros::Duration(1.0).sleep();
            // ROS_INFO("Goal Reached !");
        }
    }
}

double Track::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}

double Track::getYawFromPose(const geometry_msgs::Pose& carPose)
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

//定时器2 控制
void Track::controlLoopCB(const ros::TimerEvent&)
{

    if(flag == 1)
    {
        // while(map_path.poses.size() <= 6);
        
        if(k[3]== 0)
        {
            geometry_msgs::PoseStamped pathodom;
            // tf_listener.waitForTransform("odom","map",ros::Time::now(),ros::Duration(10.0));
            tf_listener.transformPose("odom", ros::Time(0),map_path.poses[6], "map" ,pathodom);
            double pathyaw = atan2(pathodom.pose.position.y - odom.pose.pose.position.y,pathodom.pose.position.x - odom.pose.pose.position.x);
            k[2] = pathyaw;
            k[3] = 1;
        }
        
        cmd_vel = PID(k[2]);
    }

    else if(flag == 2)
    {
        geometry_msgs::Pose carPose = odom.pose.pose;
        geometry_msgs::Twist carVel = odom.twist.twist;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        if(goal_received)
        {
            /*Estimate Steering Angle*/
            double eta = getEta(carPose);  
            if(foundForwardPt && !goal_reached)
            {
                // change the linear.x;
                cmd_vel.linear.x = realSpeed;
                cmd_vel.angular.z = eta*cmd_vel.linear.x;
            }
        }
    }
    else if(flag == 3)
    {
        double goalyaw = getYawFromPose(odom_goal.pose);
        cmd_vel = PID(goalyaw);
    }
    else 
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }
    pub_.publish(cmd_vel);
}

geometry_msgs::Twist Track::PID(double targetyaw)
{
    geometry_msgs::Twist cmd;
    cmd_vel.linear.x = 0;
    double carYaw = getYawFromPose(odom.pose.pose);
    err[0] = targetyaw - carYaw;
    if(fabs(err[0])<0.2) 
    {
        cmd.angular.z = 0;
        flag = 2;
        err[0] = 0;
        err[1] = 0;
        k[3] = 0;
        return cmd;
    }
    cmd.angular.z = k[0]*err[0]+k[1]*err[1];
    if(fabs(cmd.angular.z)>1.0) cmd.angular.z = 1.0*cmd.angular.z/fabs(cmd.angular.z);
    err[1] += err[0];
    return cmd;
}

double Track::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
    double d = sqrt(odom_car2WayPtVec.y*odom_car2WayPtVec.y+odom_car2WayPtVec.x*odom_car2WayPtVec.x);
    if(d==0) d=10000;
    double mk = 2*sin(eta)/d;
    return mk;
}

geometry_msgs::Point Track::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached){
        int map_len = map_path.poses.size();
        if(map_len < 20)
        {
            forwardPt = odom_goal_pos;
            realSpeed = 0.15;
            foundForwardPt = true;
        }

        else{
        for(int i =0; i< map_path.poses.size(); i++)
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
                        realSpeed = baseSpeed;
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
        
    }}
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");
    }

    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}

//two judging
bool Track::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
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


bool Track::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "PureTrack");
    Track track;
    track.init();
    ros::spin();
    return 0;
}
