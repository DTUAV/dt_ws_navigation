#include "../include/dt_map2d_fusion/generate_mission_point.h"
using namespace dt_map2d_fusion;
int main(int argv, char** argc) {
  ros::init(argv, argc, "test_generate_mission_point");
  generate_mission_point node;
  std::vector<pose> poses;
  pose car1Pos, car2Pos;
  car1Pos.curPosX = 2;
  car1Pos.curPosY = 4;
  car2Pos.curPosX = 5;
  car2Pos.curPosY = 4.2;
  poses.push_back(car1Pos);
  poses.push_back(car2Pos);
  map_data mapData;
  mapData.sizeX = 50;
  mapData.sizeY = 50;
  mapData.data.resize(50 * 50);
  for(int i = 0; i < 50 * 50; ++i) {
    if(i % 39 == 0) mapData.data.at(i) = -1;
    else mapData.data.at(i) = 0;
  }
  node.count_waypoints(poses, mapData);
  ros::spin();
}
