#include "../include/lidar_slam/data_endecoder.h"

std::pair<std::string, bool> pack_data(const int &id, const int &scan_size, const int &pos_size, const int &num_point, const sensor_msgs::LaserScan& scan_msg, const nav_msgs::Odometry& odom_msg) {
  if (id == 0) {
    std::string ret;
    //format 0
    //S1: 保存版本号str[0]
    std::pair<std::string, bool> tem = int_to_string(id, 1);
    if(tem.second)
      ret += tem.first;
    else
      return std::pair<std::string, bool>({}, false);

    //S2: 保存每个Range的长度str[1], 整数位为2
    tem = int_to_string(scan_size, 1);
    if (tem.second)
      ret += tem.first;
    else
      return std::pair<std::string, bool>({}, false);

    //S3: 保存位姿的位数str[2], 整数位, 小数位为2
    tem = int_to_string(pos_size, 1);
    if (tem.second)
      ret += tem.first;
    else
      return std::pair<std::string, bool>({}, false);

    //S4: 激光的点数str[3]+str[4]+str[5]+str[6]
    tem = int_to_string(num_point, 4);
    if (tem.second)
      ret += tem.first;
    else
      return std::pair<std::string, bool>({}, false);

    std::cout << "fff" << std::endl;
    //S5: range数据
    for(const auto &val : scan_msg.ranges) {
      float tem_data = val;
      tem_data = std::isfinite(tem_data) ? tem_data : (scan_msg.range_max - 0.01);//NAN, inf, -inf
      tem = range_to_string(val, scan_size - 2);
      if (tem.second)
        ret += tem.first;
      else
        return std::pair<std::string, bool>({}, false);
    }

    std::cout << "fff1" << std::endl;

    //S6: 位姿数据(6)
    //x:
    tem = pose_to_string(odom_msg.pose.pose.position.x, pos_size);
    if (tem.second)
      ret += tem.first;
    else
      return std::pair<std::string, bool>({}, false);

    //y:
    tem = pose_to_string(odom_msg.pose.pose.position.y, pos_size);
    if (tem.second)
      ret += tem.first;
    else
      return std::pair<std::string, bool>({}, false);

    //z:
    tem = pose_to_string(odom_msg.pose.pose.position.z, pos_size);
    if (tem.second)
      ret += tem.first;
    else
      return std::pair<std::string, bool>({}, false);

    //yaw:
    //四元数 -->> 欧拉角
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, quat);
    double roll, pitch, yaw;   //定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    tem = pose_to_string(yaw + 10, pos_size);//将负数都转换成正数发送，在解析时要减去10
    if (tem.second)
      ret += tem.first;
    else
      return std::pair<std::string, bool>({}, false);

    //S7: 时间戳
    ret += std::to_string(scan_msg.header.stamp.toNSec());

    int one_num = 0;
    for (const auto& val : ret) {
      one_num += one_size(val);
    }

    //S8: 奇偶校验 (1的个数为偶数)
    if (one_num % 2) {
      ret.push_back('1');
    }
    else {
      ret.push_back('0');
    }
    return std::pair<std::string, bool>(ret, true);
  }
  else {
    //format 1

  }
}

bool unpack_data(const std::string &data, sensor_msgs::LaserScan& scan_msg, nav_msgs::Odometry& odom_msg) {

  //S1: 检查数据是否正确
  std::string valid_data = data.substr(0, data.size() - 1);
  int one_num = 0;
  for (const auto &val : valid_data) {
    one_num += one_size(val);
  }
  if(one_num % 2 && data[data.size() - 1] != '1') return false;
  else if(!one_num % 2 && data[data.size() - 1] != '0') return false;

  int id = char_to_num(data[0]);//保存版本号str[0]
  if (id == 0) {
    int scan_size = char_to_num(data[1]);//每个Range的长度str[1], 整数位为2 == 一个激光距离数据的长度为scan_size
    int point_size = char_to_num(data[2]);//位姿的位数str[2], 整数位, 小数位为2 == 一个位姿点的数据势point_size + 2
    int num_point = std::stoi(data.substr(3, 4));//激光点的数目

    scan_msg.ranges.resize(num_point);

    for (int i = 0; i < num_point; ++i) {
      int start = 7 + i * scan_size;
      scan_msg.ranges[i] = string_to_range(data.substr(start, scan_size), scan_size - 2);
    }

    int odom_start = 7 + num_point * scan_size;
    //x
    odom_msg.pose.pose.position.x = string_to_pose(data.substr(odom_start, point_size + 2), point_size);
    //y
    odom_start += point_size + 2;
    odom_msg.pose.pose.position.y = string_to_pose(data.substr(odom_start, point_size + 2), point_size);
    //z
    odom_start += point_size + 2;
    odom_msg.pose.pose.position.z = string_to_pose(data.substr(odom_start, point_size + 2), point_size);
    //yaw
    odom_start += point_size + 2;
    float yaw = string_to_pose(data.substr(odom_start, point_size + 2), point_size) - 10;


    //欧拉角 -->>四元数
   tf::Quaternion q;
   q.setRPY(0, 0, yaw);

   odom_msg.pose.pose.orientation.x = q.getX();
   odom_msg.pose.pose.orientation.y = q.getY();
   odom_msg.pose.pose.orientation.z = q.getZ();
   odom_msg.pose.pose.orientation.w = q.getW();

   long long stamp = std::stoll(valid_data.substr(odom_start + point_size + 2));

   scan_msg.header.stamp.fromNSec(stamp);
   odom_msg.header.stamp.fromNSec(stamp);
  }
  else {

  }

 return true;
}

float string_to_pose(const std::string &data, const int &bit) {
  //bit为整数位
  std::string integer = data.substr(0, bit);
  std::string decimal = data.substr(bit);
  float ret = std::stof(integer) + std::stof(decimal) * std::pow(0.1, 2);
  return ret;
}

std::pair<std::string, bool> pose_to_string(const float &data, const int &bit) {
  //获取整数部分
  int data_integer = (int)data;
  // std::cout << data_integer << std::endl;
  std::vector<int> bit_data;
  while (data_integer / 10) {
    bit_data.push_back(data_integer % 10);
    data_integer /= 10;
  }
  bit_data.push_back(data_integer);

  //如果整数部分的位数都大于指定的位数，则返回空
  if(bit < bit_data.size()) {
    return std::pair<std::string, bool>({}, false);
  }

  //如果整数部分的位数小于指定的位数，以0填充
  for (int i = 0; i < bit - bit_data.size(); ++i) {
    bit_data.push_back(0);
  }

 // std::cout << "bit_data_1_size: " << bit_data.size() << std::endl;

  //获取小数部分为2位
    int data_decimal = (int)((data - (int)data) * std::pow(10, 2));
  //  std::cout << data_decimal << std::endl;
    while(data_decimal / 10) {
      bit_data.push_back(data_decimal % 10);
      data_decimal /= 10;
    }
    bit_data.push_back(data_decimal);

   // std::cout << "bit_data_2_size: " << bit_data.size() << std::endl;

    int cur_size = bit_data.size();
    for(int i = cur_size; i < bit + 2; ++i) {
      bit_data.push_back(0);
    }



    std::reverse(bit_data.begin(), bit_data.begin() + bit);
    std::reverse(bit_data.begin() + bit, bit_data.end());

  std::cout << "pose_size: " << bit_data.size() << std::endl;
  //将数据转成string
  std::string ret;
  for(const auto &val : bit_data) {
    ret.push_back(num_to_char(val));
  }
  return std::pair<std::string, bool>(ret, true);
}

float string_to_range(const std::string &data, const int &bit) {

  std::string integer = data.substr(0, 2);
  std::string decimal = data.substr(2);

  float ret = std::stof(integer) + std::stof(decimal) * std::pow(0.1, bit);
  return ret;
}

std::pair<std::string, bool> range_to_string(const float &data, const int &bit) {

  int data_integer = (int)data;
  int data_decimal = (int)((data - data_integer) * std::pow(10, bit));

 // std::cout << "data_integer" << data_integer << std::endl;
 // std::cout << "data_decimal" << data_decimal << std::endl;

  //获取整数部分的每位数字
  std::vector<int> bit_data;
  while (data_integer / 10) {
    bit_data.push_back(data_integer % 10);
    data_integer /= 10;
  }
  bit_data.push_back(data_integer);

  //整数最多有两位
  if (bit_data.size() > 2) {
    return std::pair<std::string, bool>({}, false);
  }

  int cur_size = bit_data.size();
  //如果少于两位就以0补充
  for (int i = 0; i < 2 - cur_size; ++i) {
    bit_data.push_back(0);
  }

  //std::cout << "bit_data.size" << bit_data.size() << std::endl;

  //获取小数部分的每位数字
  while (data_decimal / 10) {
    bit_data.push_back(data_decimal % 10);
    data_decimal /= 10;
  }
  bit_data.push_back(data_decimal);
  cur_size = bit_data.size();

  for (int i = cur_size; i < bit + 2; ++i) {
    bit_data.push_back(0);
  }
 // std::cout<<bit_data.size()<<std::endl;


  //翻转数据，使其符合之前的数字
  std::reverse(bit_data.begin(), bit_data.begin() + 2);
  std::reverse(bit_data.begin() + 2, bit_data.end());

  std::string ret;
  for (const auto &val : bit_data) {
    ret.push_back(num_to_char(val));
  }

  return std::pair<std::string, bool>(ret, true);
}


std::pair<std::string, bool> int_to_string(const int &data, const int &bit) {
  std::vector<int> bit_data;
  int tem_data = data;
  //获取整数的所有位数字
  while (tem_data / 10) {
    bit_data.push_back(tem_data % 10);
    tem_data /= 10;
  }
  bit_data.push_back(tem_data);

  //如果需要的位数小于所有的位数，返回空，需要保证需要的大于等于实际的
  if (bit < bit_data.size()) {
    return std::pair<std::string, bool>({}, false);
  }

  //需要位数大于实际的部分用0来填充
  for (int i = 0; i < bit - bit_data.size(); ++i) {
    bit_data.push_back(0);
  }

  //由于bit_data是反过来的，所以需要翻转
  std::reverse(bit_data.begin(), bit_data.end());

  //将每位整数转换为字符串
  std::string ret;
  for (const auto &val : bit_data) {
    ret.push_back(num_to_char(val));
  }
  return std::pair<std::string, bool>(ret, true);
}

char num_to_char(const int &data) {
  switch (data) {
  case 0: return '0';
    break;
  case 1: return '1';
    break;
  case 2: return '2';
    break;
  case 3: return '3';
    break;
  case 4: return '4';
    break;
  case 5: return '5';
    break;
  case 6: return '6';
    break;
  case 7: return '7';
    break;
  case 8: return '8';
    break;
  case 9: return '9';
  default: return ' ';
  }
}

int char_to_num(const char &data) {
  switch (data) {
  case '0': return 0;
    break;
  case '1': return 1;
    break;
  case '2': return 2;
    break;
  case '3': return 3;
    break;
  case '4': return 4;
    break;
  case '5': return 5;
    break;
  case '6': return 6;
    break;
  case '7': return 7;
    break;
  case '8': return 8;
    break;
  case '9': return 9;
    break;
  default: return -1;
  }
}


int one_size(const char& data){
  switch (data) {
  case '0': return 0;
    break;
  case '1': return 1;
    break;
  case '2': return 1;
    break;
  case '3': return 2;
    break;
  case '4': return 1;
    break;
  case '5': return 2;
    break;
  case '6': return 2;
    break;
  case '7': return 3;
    break;
  case '8': return 1;
    break;
  case '9': return 2;
    break;
  case 'A': return 2;
    break;
  case 'B': return 3;
    break;
  case 'C': return 2;
    break;
  case 'D': return 3;
    break;
  case 'E': return 3;
    break;
  case 'F': return 4;
  default: return 0;
    break;
  }
}












