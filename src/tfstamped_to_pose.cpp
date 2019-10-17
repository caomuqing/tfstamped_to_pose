#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>

ros::Publisher viconpose_pub;
void vicontf_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_to_pose");

  ros::NodeHandle nh("~");

  viconpose_pub = nh.advertise<geometry_msgs::PoseStamped>("vicon_pose", 50);

  ros::Subscriber structurepoint_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/m100/m100", 10, vicontf_cb);
  ros::spin();

  return 0;
}

void vicontf_cb(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped ps1;

  ps1.header.stamp = ros::Time::now();;
  ps1.header.frame_id = "vicon";
  ps1.pose.position.x = msg->transform.translation.x;
  ps1.pose.position.y = msg->transform.translation.y;
  ps1.pose.position.z = msg->transform.translation.z;
  ps1.pose.orientation = msg->transform.rotation;

  viconpose_pub.publish(ps1);
}
