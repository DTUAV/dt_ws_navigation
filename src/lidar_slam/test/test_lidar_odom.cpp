#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_lidar_odom");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
