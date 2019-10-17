#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <cstdio>
#include <tf/transform_datatypes.h>

ros::Publisher odom_pub;
bool heading_initialised_ = false;
bool heading_got_ = false;
double heading_radian;
tf::Quaternion heading_q;
void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg);
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);



int main(int argc, char** argv){
  ros::init(argc, argv, "odom_rotation");

  ros::NodeHandle nh("~");

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom_enu", 1000);

  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 10, odometry_cb);
  ros::Subscriber attitude_sub = nh.subscribe<sensor_msgs::Imu>("/firefly/imu", 10, imu_cb);

  ros::spin();

  return 0;
}

void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  if (!heading_got_){
    ROS_WARN("not publishing rotated odometry as rotation is not set!");
    return;
  } else heading_initialised_ = true;
  nav_msgs::Odometry odom_enu;
  geometry_msgs::Quaternion qlocal1= msg->pose.pose.orientation;
  tf::Quaternion qlocal2(qlocal1.x, qlocal1.y, qlocal1.z, qlocal1.w);
  tf::Quaternion q_enu = heading_q*qlocal2;
  odom_enu.header.stamp = ros::Time::now();;
  odom_enu.header.frame_id = "world";
  odom_enu.pose.pose.orientation.x = q_enu.x();
  odom_enu.pose.pose.orientation.y = q_enu.y();
  odom_enu.pose.pose.orientation.z = q_enu.z();
  odom_enu.pose.pose.orientation.w = q_enu.w();

  odom_enu.pose.pose.position.x = msg->pose.pose.position.x*cos(heading_radian)-
                                  msg->pose.pose.position.y*sin(heading_radian);
  odom_enu.pose.pose.position.y = msg->pose.pose.position.x*sin(heading_radian)+
                                  msg->pose.pose.position.y*cos(heading_radian);
  odom_enu.pose.pose.position.z = msg->pose.pose.position.z;                                
  odom_enu.twist.twist.linear.x = msg->twist.twist.linear.x*cos(heading_radian)-
                                  msg->twist.twist.linear.y*sin(heading_radian);
  odom_enu.twist.twist.linear.y = msg->twist.twist.linear.x*sin(heading_radian)+
                                  msg->twist.twist.linear.y*cos(heading_radian);
  odom_enu.twist.twist.linear.y = msg->twist.twist.linear.z;
  odom_pub.publish(odom_enu);
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (!heading_initialised_){
    tf::Quaternion qb(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    heading_radian = tf::getYaw(qb);
    tf::Quaternion qf = tf::createQuaternionFromRPY(0, 0, heading_radian);
    heading_q = tf::Quaternion(qf.x(), qf.y(), qf.z(), qf.w());
    heading_got_ = true;
  }
}