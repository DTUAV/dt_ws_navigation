#ifndef GENERATE_MISSION_POINT_H
#define GENERATE_MISSION_POINT_H
#include "ros/ros.h"
#include "dt_message_package/waypoints.h"
#include "common.h"
#include "std_msgs/Bool.h"

namespace dt_map2d_fusion {

class generate_mission_point
{
public:
  generate_mission_point();
  void count_waypoints(const std::vector<pose> &poses, const map_data &fusionMap);
  std::vector<waypoint> get_waypoints();
  bool getIsFinish();

private:
  std::vector<waypoint> allWaypoints;
  int pointsNum; //the number of waypoints for each count
  float startPosX; //the start position of x
  float startPosY; //the start position of y
  float startPosZ; //the start position of z
  int mapSizeX; //the map size of x
  int mapSizeY; //the map size of y
  float mapScale; //the map scale
  bool isFinish;
  float scanRange;

  ros::Publisher waypointsPub;
  ros::Publisher missionStatusPub;
};
}

#endif // GENERATE_MISSION_POINT_H
