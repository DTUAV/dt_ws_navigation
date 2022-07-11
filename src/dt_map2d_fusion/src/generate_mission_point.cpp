#include "../include/dt_map2d_fusion/generate_mission_point.h"
using namespace dt_map2d_fusion;

generate_mission_point::generate_mission_point()
{
  ros::NodeHandle nh("~");
  nh.param<int>("map_size_x", mapSizeX, 50);
  nh.param<int>("map_size_y", mapSizeY, 50);
  nh.param<float>("start_pos_x", startPosX, 0);
  nh.param<float>("start_pos_y", startPosY, 0);
  nh.param<float>("start_pos_z", startPosZ, 0);
  nh.param<int>("points_num", pointsNum, 10);
  nh.param<float>("map_resolution", mapScale, 0.1);
  nh.param<float>("scan_range", scanRange, 30.0);

  isFinish = false;

}

static bool greater_sort(empty_point_info a, empty_point_info b) {
  return a.containNum >= b.containNum;
}

static bool less_sort(waypoint_div_info a, waypoint_div_info b) {
  return a.distance <= b.distance;
}

std::vector<waypoint> generate_mission_point::get_waypoints() {
  return allWaypoints;
}

void generate_mission_point::count_waypoints(const std::vector<pose> &poses, const map_data &fusionMap) {

  if(poses.empty()) {
    ROS_WARN("dt_map2d_fusion-generate_mission_point: the object poses are empty");
    return;
  }
  // world points to grid points
  int waypointNum = poses.size();
  allWaypoints.resize(waypointNum);
  std::vector<gridXY> gridPoses;
  gridPoses.resize(waypointNum);
  for(int i = 0; i < waypointNum; ++i) {
    gridXY tem;
    tem.gridX = floor(poses.at(i).curPosX / mapScale + 0.5) + 0.5 * mapSizeX;
    tem.gridY = floor(poses.at(i).curPosY / mapScale + 0.5) + 0.5 * mapSizeY;
    gridPoses.at(i) = tem;

    std::cout<< "gridPose " << i << ": ( " << tem.gridX << ", " << tem.gridY << " )" << std::endl;
  }
  //find empty grid
  int gridMapData[mapSizeX][mapSizeY];
  std::vector<gridXY> emptyData;
  //map to 2d matrix
  for(int i = 0; i < mapSizeX * mapSizeY; ++i) {
    gridMapData[i / mapSizeY][i - (i / mapSizeY) * mapSizeX] = fusionMap.data.at(i);
    if(fusionMap.data.at(i) == -1){
      gridXY tem;
      tem.gridX = i / mapSizeX;
      tem.gridY = i - (i / mapSizeY) * mapSizeX;
      emptyData.push_back(tem);

      std::cout<< "emptyData " << i << ": ( " << tem.gridX << ", " << tem.gridY << " )" << std::endl;
    }
  }
  if(emptyData.empty()) {
    ROS_INFO("All environment have been exprolated");
    isFinish = true;
    return;
  }
  //let each point to center and detected some points with const range
  std::vector<empty_point_info> cirsInfo;
  for(int i = 0; i < emptyData.size(); i++) {
    empty_point_info tem;
    tem.mapIndex.gridX = emptyData.at(i).gridX;
    tem.mapIndex.gridY = emptyData.at(i).gridY;
    int sum = 0;
    for(int j = 0; j < emptyData.size(); j++) {
      if(abs(emptyData.at(j).gridX - tem.mapIndex.gridX) <= scanRange && abs(emptyData.at(j).gridY - tem.mapIndex.gridY) <= scanRange) sum++;
    }
    tem.containNum = sum;
    cirsInfo.push_back(tem);

    std::cout << "empty_point_info " << i << ": ( " << tem.mapIndex.gridX << ", " << tem.mapIndex.gridY << " )" << "containNum: " << tem.containNum << std::endl;
  }
  //find n data;
  sort(cirsInfo.begin(), cirsInfo.end(), greater_sort);
  std::vector<gridXY> gridWaypoints;
  if(cirsInfo.size() > pointsNum) {
    for(int i = 0; i < pointsNum; ++i) {
      gridWaypoints.push_back(cirsInfo.at(i).mapIndex);
    }
  }
  else {
    for(int i = 0; i < cirsInfo.size(); ++i) {
      gridWaypoints.push_back(cirsInfo.at(i).mapIndex);
    }
  }

  {
    for(auto val : gridWaypoints)
    std::cout << "n_data_info: ( " << val.gridX << ", " << val.gridY << " )" << std::endl;
  }



  //divide waypoints
  std::vector<waypoint_div_info> waypointsDivInfo;
  for(int i = 0; i < gridWaypoints.size(); ++i) {
    waypoint_div_info info;
    info.waypoint.gridX = gridWaypoints.at(i).gridX;
    info.waypoint.gridY = gridWaypoints.at(i).gridY;
    for(int j = 0; j < gridPoses.size(); j++) {
      info.id = j;
      info.distance = sqrt((gridPoses.at(j).gridX - info.waypoint.gridX) * (gridPoses.at(j).gridX - info.waypoint.gridX)
                           + (gridPoses.at(j).gridY - info.waypoint.gridY) * (gridPoses.at(j).gridY - info.waypoint.gridY));
      waypointsDivInfo.push_back(info);
    }

    {
      for(auto val : waypointsDivInfo) {
        std::cout<<"before sort id: " << val.id <<std::endl;
      }
    }

    sort(waypointsDivInfo.begin(), waypointsDivInfo.end(), less_sort);

    {
      for(auto val : waypointsDivInfo) {
        std::cout<<"after sort id: " << val.id <<std::endl;
      }
    }

    allWaypoints.at(waypointsDivInfo.at(0).id).xs.push_back(waypointsDivInfo.at(0).waypoint.gridX);
    allWaypoints.at(waypointsDivInfo.at(0).id).ys.push_back(waypointsDivInfo.at(0).waypoint.gridY);
    allWaypoints.at(waypointsDivInfo.at(0).id).zs.push_back(0);
    waypointsDivInfo.clear();
  }
  //grid points to world points
  for(auto &val : allWaypoints) {
    for(auto &temX : val.xs) temX = (temX - mapSizeX / 2) * mapScale + startPosX;
    for(auto &temY : val.ys) temY = (temY - mapSizeY / 2) * mapScale + startPosY;
  }

  {
    for(int i = 0; i < allWaypoints.size(); ++i) {
      for(int j = 0; j < allWaypoints.at(i).xs.size(); ++j) {
        std::cout << "allWaypoints: " << i << ": ( " << allWaypoints.at(i).xs.at(j) << ", " << allWaypoints.at(i).ys.at(j) << ")" << std::endl;
      }
    }
  }
}



























