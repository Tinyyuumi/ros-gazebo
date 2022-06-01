
#include <iostream>
#include <vector>
#include <set>
#include <random>
#include "ros/ros.h"
#include <map>
#include <string>
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <boost/random.hpp>		//随机数所需头文件
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
class GPS
{
  public:
      void start();

  private:
      ros::NodeHandle n;
      ros::Subscriber odom_sub;
      ros::Publisher pub;
      nav_msgs::Odometry temp;
      void Callback(const nav_msgs::Odometry::ConstPtr& msg);
      
};


void GPS::start()
{
  odom_sub = n.subscribe("/odom", 1, &GPS::Callback, this);
  pub = n.advertise<nav_msgs::Odometry>("/robot1/gps_sim", 1000);

}


void GPS::Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double mean = 0.05;
    double stddiv = 0.01;
    boost::mt19937 zgy;								//等分布均匀伪随机数发生器
    zgy.seed(static_cast<unsigned int>(time(0)));	//随机种子
    boost::normal_distribution<> nd(mean, stddiv);	//定义正态分布，均值为mu，标准差为sigma
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> gauss_noise(zgy, nd);	//生成高斯噪声
    
    temp = *msg;
    temp.pose.pose.position.x = temp.pose.pose.position.x + static_cast<float> (gauss_noise());
    temp.pose.pose.position.y = temp.pose.pose.position.y + static_cast<float> (gauss_noise());

    temp.pose.covariance[0] = 0.0001;
    temp.pose.covariance[7] = 0.0001;
    temp.pose.covariance[14] = 0.0001;
    temp.pose.covariance[21] = 0.00000001;
    temp.pose.covariance[28] = 0.00000001;
    temp.pose.covariance[35] = 0.00000001;

    pub.publish(temp);
        
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "MyGPS");
  GPS gps;
  gps.start();
  ros::spin();
  return 0;
}
