#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>
#include <math.h>
#include <iostream>
#include <string>
using namespace std;

class Obscale
{
    public:
        void start();

    private:
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        ros::Subscriber twist_sub;

        gazebo_msgs::SetModelState set_model_state_srv;
        gazebo_msgs::ModelState des_model_state;
        geometry_msgs::Twist twist;
        geometry_msgs::Pose pose;
        geometry_msgs::Quaternion quat;
        void callback(const geometry_msgs::Twist::ConstPtr& twistMsg);
};

void Obscale::start()
{
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = 0.0;
    quat.w = 1.0;

    pose.position.x = -5;
    pose.position.y = 10;
    pose.position.z = 6.2;
    pose.orientation= quat;

    des_model_state.model_name = "my_box";
    des_model_state.pose = pose;
    des_model_state.twist = twist;
    des_model_state.reference_frame = "world";

    twist_sub = n.subscribe("my/cmd_vel", 1, &Obscale::callback, this);
}

void Obscale::callback(const geometry_msgs::Twist::ConstPtr& twistMsg)
{
    twist = *twistMsg;
    if(twist.linear.x > 0) pose.position.y += 0.2;
    else if(twist.linear.x < 0) pose.position.y -= 0.2;
    else if(twist.linear.y > 0) pose.position.x -= 0.2;
    else if(twist.linear.y < 0) pose.position.x += 0.2;
    else;

    des_model_state.pose = pose;
    set_model_state_srv.request.model_state = des_model_state;
    client.call(set_model_state_srv);
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "move_gazebo_model");
    Obscale myob;
    myob.start();
    ros::spin();
    return 0;
}


