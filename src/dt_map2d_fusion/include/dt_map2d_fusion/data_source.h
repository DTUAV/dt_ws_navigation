#ifndef DATA_SOURCE_H
#define DATA_SOURCE_H
#include "dt_common/define_common.h"
#include "vector"
#include "ros/ros.h"
#include "dt_message_package/CloudMessage.h"
#include "nav_msgs/OccupancyGrid.h"

namespace dt_map2d_fusion {

typedef struct {
  int sizeX;
  int sizeY;
  std::vector<int> data;
} map_data;

typedef struct {
  float curPosX;
  float curPosY;
  float curPosZ;
  float curRotX;
  float curRotY;
  float curRotZ;
  float curRotW;
} pose;

typedef struct {
  int gridX;
  int gridY;
} gridXY;

typedef struct {
  gridXY mapIndex;
  int containNum;
} empty_point_info;

typedef struct {
  gridXY waypoint;
  int distance;
  int id;
} waypoint_div_info;

typedef struct {
  vector<float> xs;
  vector<float> ys;
  vector<float> zs;
} waypoint;

class data_source
{
public:
  data_source();
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


};
}
#endif // DATA_SOURCE_H
