#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf/transform_datatypes.h"//转换函数头文件
#include <iostream>
#include <fstream>
using namespace std;
ofstream pose,amcl;
tf::Quaternion pose_q,amcl_q;


// 接收到订阅消息后，会进入消息回调函数
void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    tf::quaternionMsgToTF(odom.pose.pose.orientation, pose_q);
 
    double roll, pitch, yaw;//定义存储roll,pitch and yaw的容器
    tf::Matrix3x3(pose_q).getRPY(roll, pitch, yaw); //进行转换

    pose<<yaw<<"\n";
}

void amclCallback(const turtlesim::Pose::ConstPtr& msg)
{
	tf::quaternionMsgToTF(odom.pose.pose.orientation, amcl_q);

    double roll, pitch, yaw;//定义存储roll,pitch and yaw的容器
    tf::Matrix3x3(amcl_q).getRPY(roll, pitch, yaw); //进行转换
	amcl<<yaw<<"\n";
}

int main(int argc,char **argv)
{
	//初始化ROS节点
	ros::init(argc,argv,"pose_subscriber");

	//创建节点语柄
	ros::NodeHandle n;
	
	pose.open("pose.txt",ios::trunc); //ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建
	amcl.open("amcl.txt",ios::trunc);

	
	// 创建一个Subscriber，订阅名为/turtle/pose的topic,注册回调函数poseCallback
	ros::Subscriber pose_sub=n.subscribe("/pose",10,poseCallback);

	ros::Subscriber pose_sub=n.subscribe("/amcl_pose",10,amclCallback);
	// 循环等待回调函数
	ros::Rate loop_rate(1);
	int count = 0;

	while( ros::ok() )
	{
		loop_rate.sleep();
	}

	pose.close();
	amcl.close();
	return 0;
}
