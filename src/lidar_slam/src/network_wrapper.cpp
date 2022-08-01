#include "../include/lidar_slam/network_wrapper.h"

network_send_wrapper::~network_send_wrapper() {
  delete odom_sub;
  delete scan_sub;
  delete sync;
}

network_send_wrapper::network_send_wrapper() {

  ros::NodeHandle nh("~");

  nh.param<int>("source_id", source_id, 1);
  nh.param<int>("target_id", target_id, 101);

  nh.param<int>("format_id", format_id, 0);
  nh.param<int>("scan_size", scan_size, 4);//激光的数据为整数2位，小数为2两位，厘米精度
  nh.param<int>("pos_size", pos_size, 2);//位姿的整数部分在两位数以内

  std::string odom_sub_topic = "/odom";
  nh.param<std::string>("odom_sub_topic", odom_sub_topic, "/odom");

  std::string scan_sub_topic = "/scan";
  nh.param<std::string>("scan_sub_topic", scan_sub_topic, "/scan");

  std::string network_pub_topic = "/network_pub";
  nh.param<std::string>("network_pub_topic", network_pub_topic, "/network_pub");

  odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_sub_topic, 1);
  scan_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, scan_sub_topic, 1);
  sync = new message_filters::Synchronizer<sensor_policy>(sensor_policy(10), *scan_sub, *odom_sub);
  sync->registerCallback(boost::bind(&network_send_wrapper::message_callback, this, _1, _2));

  network_data_pub = nh.advertise<dt_message_package::CloudMessage>(network_pub_topic, 1);

}

network_send_wrapper::message_callback(const sensor_msgs::LaserScanConstPtr &scan_msg, const nav_msgs::OdometryConstPtr &odom_msg) {

  dt_message_package::CloudMessage send_msg;
  send_msg.SourceID = source_id;
  send_msg.TargetID = target_id;
  send_msg.MessageID = Lidar2dMessageID;
  send_msg.TimeStamp = ros::Time::now().toNSec();

  DTUAV::Lidar2dMessage data_msg;

  std::pair<std::string, bool> pack_data = pack_data(format_id, scan_size, pos_size, scan_msg->ranges.size(), *scan_msg, *odom_msg);
  if (pack_data.second) {
    std::string cpr_data;
    if (!util::Util::CompressString(pack_data.first, cpr_data)) {
      data_msg.data = cpr_data;
      send_msg.MessageData = x2struct::X::tojson(data_msg);
      network_data_pub.publish(send_msg);
    }
    else
      std::cerr << "compress data fail" << std::endl;
  }
  else {
    std::cerr << "pack_data fail" << std::endl;
  }

}



network_recv_wrapper::network_recv_wrapper() {

  ros::NodeHandle nh("~");
  std::string scan_pub_topic = "/scan";
  nh.param<std::string>("scan_pub_topic", scan_pub_topic, "/scan");

  std::string odom_pub_topic = "/odom";
  nh.param<std::string>("odom_pub_topic", odom_pub_topic, "/odom");

  std::string network_data_sub_topic = "/network_data_sub";
  nh.param<std::string>("network_data_sub_topic", network_data_sub_topic, "/network_data_sub");

  nh.param<float>("angle_increment", angle_increment, 0.005);
  nh.param<float>("angle_max", angle_max, 6.28318);
  nh.param<float>("angle_min", angle_min, 0);
  nh.param<float>("range_max", range_max, 10.0);
  nh.param<float>("range_min", range_min, 0.02);


  scan_msg.angle_increment = angle_increment;
  scan_msg.angle_max = angle_max;
  scan_msg.angle_min = angle_min;
  scan_msg.range_min = range_min;
  scan_msg.range_max = range_max;

  scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_pub_topic, 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_pub_topic, 1);

  network_data_sub = nh.subscribe(network_data_sub_topic, 1, &network_recv_wrapper::network_data_sub_callback, this);

}

network_recv_wrapper::network_data_sub_callback(const dt_message_package::CloudMessageConstPtr &msg) {
  if (msg.get()->MessageID == Lidar2dMessageID) {
    DTUAV::Lidar2dMessage netwok_msg;
    bool is_load = x2struct::X::loadjson(msg.get()->MessageData, netwok_msg, false);
    if (is_load) {
      std::string dep_msg;
      if (!util::Util.DecompressString(netwok_msg.data, dep_msg)) {
        if (unpack_data(dep_msg, scan_msg, odom_msg)) {
          scan_pub.publish(scan_msg);
          odom_pub.publish(odom_msg);
        }
      }
    }
  }
}

