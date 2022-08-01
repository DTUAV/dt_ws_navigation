#ifndef NETWORK_WRAPPER_H
#define NETWORK_WRAPPER_H
#include "dt_common/define_common.h"
#include "dt_message_package/CloudMessage.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "data_endecoder.h"
#include "util.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include "ros/ros.h"

#include "x2struct/x2struct.hpp"

class network_send_wrapper {
public:
  network_send_wrapper();
  ~network_send_wrapper();
  void message_callback(const sensor_msgs::LaserScanConstPtr& scan_msg, const nav_msgs::OdometryConstPtr& odom_msg);

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,nav_msgs::Odometry> sensor_policy;
  ros::Publisher network_data_pub;
  message_filters::Subscriber<sensor_msgs::LaserScan>* scan_sub;
  message_filters::Subscriber<nav_msgs::Odometry>* odom_sub;
  message_filters::Synchronizer<sensor_policy>* sync;
  int source_id;
  int target_id;
  int format_id;//版本号
  int scan_size;//每个Range的长度str[1], 整数位为2
  int pos_size;//保存位姿的整数位数str[2], 小数位为2

};

class network_recv_wrapper {
public:
  network_recv_wrapper();
  void network_data_sub_callback(const dt_message_package::CloudMessageConstPtr& msg);

private:
  ros::Publisher scan_pub;
  ros::Publisher odom_pub;
  ros::Subscriber network_data_sub;

  sensor_msgs::LaserScan scan_msg;
  nav_msgs::Odometry odom_msg;

  float angle_increment;
  float angle_max;
  float angle_min;
  float range_max;
  float range_min;

};

#endif // NETWORK_WRAPPER_H
