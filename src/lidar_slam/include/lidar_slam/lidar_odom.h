#ifndef LIDAR_ODOM_H
#define LIDAR_ODOM_H
//这个文件主要完成工作： 通过相邻帧的激光雷达数据估计出机器人当前的位姿，有矩阵计算和优化两种方法可供配置
//参考资料: https://blog.csdn.net/tiancailx/article/details/110830731?spm=1001.2101.3001.6650.7&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-7-110830731-blog-90373612.pc_relevant_aa&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-7-110830731-blog-90373612.pc_relevant_aa&utm_relevant_index=14
//ros的头文件
#include <ros/ros.h>

//ros的消息头文件
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

//pcl库头文件
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//C++容器头文件
#include <vector>

//eigen矩阵库
#include "eigen3/Eigen/Eigen"

//icp
#include "icp2d.h"

class lidar_odom
{
public:
  lidar_odom();
  void scan_sub_callback(const sensor_msgs::LaserScanConstPtr& msg);


private:
  ros::Subscriber scan_sub;
  ros::Publisher pointcloud2_pub;
  ros::Publisher odom_pub;

  icp2d icp2d_node;

  bool is_init;


};

#endif // LIDAR_ODOM_H
