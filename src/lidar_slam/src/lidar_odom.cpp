#include "../include/lidar_slam/lidar_odom.h"

lidar_odom::lidar_odom()
{
   ros::NodeHandle nh("~");

   std::string scan_sub_topic = "/scan";
   nh.param<std::string>("scan_sub_topic", scan_sub_topic);

   std::string pointcloud2_pub_topic = "/pointcloud2";
   nh.param<std::string>("pointcloud2_pub_topic", pointcloud2_pub_topic);

   std::string odom_pub_topic = "/odom";
   nh.param<std::string>("odom_pub_topic", odom_pub_topic);

   scan_sub = nh.subscribe(scan_sub_topic, 2, lidar_odom::scan_sub_callback, this);
   pointcloud2_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud2_pub_topic, 1);
   odom_pub = nh.advertise<nav_msgs>(odom_pub_topic, 1);

   is_init = false;

}

void lidar_odom::scan_sub_callback(const sensor_msgs::LaserScanConstPtr &msg) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
  cloud_msg.get()->points.resize(msg.get()->ranges.size());

  std::vector<Eigen::Vector2f> data;

  for(int i = 0; i < msg.get()->ranges.size(); ++i) {
    pcl::PointXYZ& point_tmp = cloud_msg->points[i];
    float range = msg.get()->ranges[i];
    if(!std::isfinite(range)) {
      point_tmp.x = std::numeric_limits<float>::quiet_NaN();
      point_tmp.y = std::numeric_limits<float>::quiet_NaN();
      point_tmp.z = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
    if(range > msg->angle_min && range < msg->range_max) {

      float angle = msg->angle_max + i * msg->angle_increment;
      point_tmp.x = range * std::cos(angle);
      point_tmp.y = range * std::sin(angle);
      point_tmp.z = 0.0;

      Eigen::Vector2f tem;
      tem << point_tmp.x, point_tmp.y;

      data.push_back(tem);

    }
    else {
      point_tmp.x = std::numeric_limits<float>::quiet_NaN();
      point_tmp.y = std::numeric_limits<float>::quiet_NaN();
      point_tmp.z = std::numeric_limits<float>::quiet_NaN();
    }
  }
  cloud_msg->width = msg->ranges.size();
  cloud_msg->height = 1;
  cloud_msg->is_dense = false;
  pcl_conversions::toPCL(msg->header, cloud_msg->header);
  pointcloud2_pub.publish(cloud_msg);

  if(!is_init) {
    icp2d_node.init(data);
  }
  else {
    icp2d_node.update(data);
  }

}
