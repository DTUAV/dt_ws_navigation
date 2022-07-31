#ifndef DATA_ENDECODER_H
#define DATA_ENDECODER_H
#include "string"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "utility"
#include <algorithm>
#include <math.h>
#include <tf/tf.h>

std::pair<std::string, bool> pack_data(const int &id, const int &scan_size, const int &pos_size, const int &num_point, const sensor_msgs::LaserScanConstPtr& scan_msg, const nav_msgs::OdometryConstPtr& odom_msg);
bool uppack_data(const std::string &data, sensor_msgs::LaserScan::Ptr &scan_msg, nav_msgs::Odometry::Ptr &odom_msg);

//将一个位置浮点数转换为一个字符串（bit为整数位，小数位为2位）
std::pair<std::string, bool> pose_to_string(const float &data, const int &bit);
float string_to_pose(const std::string& data, const int &bit);


//将一个距离浮点数转换为一个字符串（两位整数和bit位小数）
std::pair<std::string, bool> range_to_string(const float &data, const int &bit);
float string_to_range(const std::string& data, const int &bit);

//将一个整数保存到size等于bit的字符串中并返回
std::pair<std::string, bool> int_to_string(const int &data, const int &bit);

char num_to_char(const int &data);
int char_to_num(const char &data);

int one_size(const char& data);
#endif // DATA_ENDECODER_H
