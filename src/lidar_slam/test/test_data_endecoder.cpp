#include "../include/lidar_slam/data_endecoder.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "../include/lidar_slam/util.h"

int main(int argv, char **argc) {

  ros::init(argv, argc, "test");
  ros::NodeHandle nh("~");
  int id = 0;
  int scan_size = 4;
  int pos_size = 2;
  int num_point = 1080;
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.stamp = ros::Time::now();
  nav_msgs::Odometry odom_msg;
  std::cout<<"dddd1"<<std::endl;
  scan_msg.ranges.resize(1080);
  for(int i = 0; i < 1080; ++i) {
    scan_msg.ranges.at(i) = i * 0.05 + 0.12;
  }

  odom_msg.pose.pose.position.x = 10.23;
  odom_msg.pose.pose.position.y = 11.2;
  odom_msg.pose.pose.position.z = 0.55;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.5;
  odom_msg.pose.pose.orientation.w = 0.866;

  std::cout<<"dddd"<<std::endl;

  std::pair<std::string, bool> data = pack_data(id, scan_size, pos_size, num_point, scan_msg, odom_msg);
  std::cout<<data.second<<std::endl;
  std::cout<<data.first<<std::endl;
  std::cout<< "sizeof(msg): " << data.first.size() << std::endl;
  std::string dst;
  int res = util::Util::CompressString(data.first, dst);
  std::cout << "is_success : " << res << std::endl;
 // std::cout << "compress_result: " << dst << std::endl;
  std::cout << "sizeof(dst)" << dst.size() << std::endl;

  std::string de_dest;
  int de_res = util::Util::DecompressString(dst, de_dest);
  std::cout << "de_result: " << de_dest << std::endl;
  std::cout << "de_size: " << de_dest.size() << std::endl;

  sensor_msgs::LaserScan scan_result;
  nav_msgs::Odometry odom_result;
  bool unpack_res = unpack_data(de_dest, scan_result, odom_result);

  for(int i = 0; i < scan_result.ranges.size(); ++i) {
    std::cout << scan_result.ranges[i] << ",";
  }
  std::cout << std::endl;

  std::cout << "x: " << odom_msg.pose.pose.position.x << std::endl;
  std::cout << "y: " << odom_msg.pose.pose.position.y << std::endl;
  std::cout << "z: " << odom_msg.pose.pose.position.z << std::endl;

  std::cout << "rotation_x: " << odom_msg.pose.pose.orientation.x << std::endl;
  std::cout << "rotation_y: " << odom_msg.pose.pose.orientation.y << std::endl;
  std::cout << "rotation_z: " << odom_msg.pose.pose.orientation.z << std::endl;
  std::cout << "rotation_w: " << odom_msg.pose.pose.orientation.w << std::endl;

  std::cout << "stamp: " << scan_msg.header.stamp.toNSec() << std::endl;


  return 1;
}
