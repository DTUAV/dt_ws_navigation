#ifndef DATA_SOURCE_H
#define DATA_SOURCE_H
#include "dt_common/define_common.h"
#include "vector"
#include "ros/ros.h"
#include "dt_message_package/CloudMessage.h"
#include "nav_msgs/OccupancyGrid.h"
#include "generate_mission_point.h"
#include "common.h"
namespace dt_map2d_fusion {
class data_source
{
public:
  data_source();
  ~data_source();
  void cloud_msg_sub_cb(const dt_message_package::CloudMessagePtr& msg);
  void map_fusion_timer_cb(const ros::TimerEvent& event);
  void test_data_source();
private:
  int mapSizeX; //the size x of map data
  int mapSizeY; //the size y of map data
  int mapNum; //the number of map data (number of data source in)
  float mapFusionHz;
  float mapResolution;
  std::vector<map_data> maps; //the all map data
  std::vector<pose> poses; //the all object current pose
  map_data fusionMap; //the fusion map including all other map data;
  ros::Subscriber cloudMsgSub;
  ros::Publisher fusionMapPub;
  ros::Timer mapFusionTimer;
  nav_msgs::OccupancyGrid fusionMapDataMsg;
  dt_map2d_fusion::generate_mission_point *generatePointNode;

};
}
#endif // DATA_SOURCE_H
