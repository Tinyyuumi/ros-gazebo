/********************/
/* CLASS DEFINITION */
/********************/
class Controller
{
    public:
        Controller();
        double getEta(const geometry_msgs::Pose& carPose);
        double getYawFromPose(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getfirstpathyaw(const geometry_msgs::Pose& carPose,double yaw);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);


    private:
        ros::NodeHandle n_;
        double R;
        double baseSpeed,goalRadius;
        int controller_freq;

        ros::Subscriber odom_sub, path_sub, goal_sub;


        tf::TransformListener tf_listener;
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path, odom_path;

        bool foundForwardPt,goal_received, goal_reached;
        int car_stop, forward_dis;

        ros::Publisher pub_;
        geometry_msgs::Twist cmd_vel;
        ros::Timer timer1, timer2;
        geometry_msgs::Point odom_goal_pos;

        // three callback function
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void goalReachingCB(const ros::TimerEvent&);
        // void controlLoopCB();
        void controlLoopCB(const ros::TimerEvent&);

}; // end of class

