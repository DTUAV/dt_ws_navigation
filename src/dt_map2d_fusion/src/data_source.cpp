#include "../include/dt_map2d_fusion/data_source.h"
using namespace dt_map2d_fusion;

data_source::~data_source() {
  delete generatePointNode;
}

data_source::data_source() : mapSizeX(50), mapSizeY(50), mapNum(2)
{
  ros::NodeHandle nh("~");
  nh.param<int>("map_size_x", mapSizeX, 50);
  nh.param<int>("map_size_y", mapSizeY, 50);
  nh.param<int>("map_num", mapNum, 2);
  nh.param<float>("map_fusion_hz", mapFusionHz, 1.0);
  nh.param<float>("map_resolution", mapResolution, 0.5);
  std::string cloudMsgSubTopic = "/cloud_msg/sub";
  nh.param<std::string>("cloud_msg_sub_topic", cloudMsgSubTopic,"/cloud_msg/sub");
  std::string fusionMapPubTopic = "/fusion_map/pub";
  nh.param<std::string>("fusion_map_pub", fusionMapPubTopic, "/fusion_map/pub");
  maps.resize(mapNum);
  poses.resize(mapNum);
  fusionMap.sizeX = mapSizeX;
  fusionMap.sizeY = mapSizeY;
  fusionMap.data.resize(mapSizeX * mapSizeY);
  for(auto &val : fusionMap.data) val = -1;

  fusionMapDataMsg.info.height = mapSizeY;
  fusionMapDataMsg.info.width = mapSizeX;
  fusionMapDataMsg.info.origin.position.x = 0;
  fusionMapDataMsg.info.origin.position.y = 0;
  fusionMapDataMsg.info.origin.position.z = 0;
  fusionMapDataMsg.info.origin.orientation.x = 0;
  fusionMapDataMsg.info.origin.orientation.y = 0;
  fusionMapDataMsg.info.origin.orientation.z = 0;
  fusionMapDataMsg.info.origin.orientation.w = 1;
  fusionMapDataMsg.info.resolution = mapResolution;
  fusionMapDataMsg.header.frame_id = "map";
  fusionMapDataMsg.data.resize(mapSizeX * mapSizeY);

  cloudMsgSub = nh.subscribe(cloudMsgSubTopic, 1, &data_source::cloud_msg_sub_cb, this);
  fusionMapPub = nh.advertise<nav_msgs::OccupancyGrid>(fusionMapPubTopic, 1);
  mapFusionTimer = nh.createTimer(ros::Duration(1 / mapFusionHz), &data_source::map_fusion_timer_cb, this);

  generatePointNode = new generate_mission_point();

  test_data_source();
}

void data_source::test_data_source() {
  //one half is 255, one half is 0
  map_data mapData1, mapData2;
  mapData1.sizeX = mapSizeX;
  mapData1.sizeY = mapSizeY;
  mapData2.sizeX = mapSizeX;
  mapData2.sizeY = mapSizeY;
  mapData1.data.resize(mapSizeX * mapSizeY);
  mapData2.data.resize(mapSizeX * mapSizeY);
  for(auto &val : mapData1.data) val = -1;
  for(auto &val : mapData2.data) val = -1;

  for(int i = 0; i < (mapSizeX * mapSizeY) / 2; i++) {
    if(i < mapSizeX * 2)
      mapData1.data.at(i) = 10;
    else mapData1.data.at(i) = 0;
      mapData2.data.at(i + (mapSizeX * mapSizeY) / 2) = 100;
  }
  maps.at(0) = mapData1;
  maps.at(1) = mapData2;
}

void data_source::map_fusion_timer_cb(const ros::TimerEvent &event) {

  //check all maps have been reveived
  for(auto val : maps) {
    if(val.data.empty()) return;
    if(val.data.size() != val.sizeX * val.sizeY) {
      ROS_WARN("dt_map2d_fusion-data_source: the map data is no equal to sizeX * sizeY");
      return;
    }
  }
  //fuse map
  // 255 and 0 is true data =>update, 127 no update
  for(const auto &val : maps) {
    for(int i = 0; i < val.data.size(); ++i) {
      if(val.data.at(i) != -1) fusionMap.data.at(i) = val.data.at(i);
    }
  }
  //publish the fusion map
  fusionMapDataMsg.header.stamp = ros::Time::now();
  for(int i = 0; i < fusionMapDataMsg.data.size(); i++) {
    fusionMapDataMsg.data.at(i) = fusionMap.data.at(i);
  }
  fusionMapPub.publish(fusionMapDataMsg);
}

void data_source::cloud_msg_sub_cb(const dt_message_package::CloudMessagePtr& msg) {

  if(msg.get()->SourceID - R_CAR_1 >= mapNum) {
    ROS_WARN("dt_map2d_fusion-data_source: the config data source is less than actual received data");
    return;
  }
  switch(msg.get()->MessageID) {
  case CompressedImageMessageID: {
    DTUAV::CompressedImageMessage dataMsg;
    bool is_load = x2struct::X::loadjson(msg.get()->MessageData, dataMsg, false);
    if(is_load) {
      map_data mapData;
      for(auto &val : dataMsg.data) {
        if(val == 255) val = 0;
        if(val == 127) val = -1;
        if(val == 0) val = 100;
      }
      mapData.data = dataMsg.data;
      mapData.sizeX = mapSizeX;
      mapData.sizeY = mapSizeY;
      maps.at(msg.get()->SourceID - R_CAR_1) = mapData;
    }
    break;
  }
  case CurrentLocalPositionMsgID: {
    DTUAV::CurrentLocalPositionMsg poseMsg;
    bool is_load = x2struct::X::loadjson(msg.get()->MessageData, poseMsg, false);
    if(is_load) {
      pose posData;
      posData.curPosX = poseMsg.position_x;
      posData.curPosY = poseMsg.position_y;
      posData.curPosZ = poseMsg.position_z;
      posData.curRotX = poseMsg.rotation_x;
      posData.curRotY = poseMsg.rotation_y;
      posData.curRotZ = poseMsg.rotation_z;
      posData.curRotW = poseMsg.rotation_w;
      poses.at(msg.get()->SourceID - R_CAR_1) = posData;
    }
    break;
  }
  case FinishOneMissionMessageID: {
    DTUAV::FinishOneMissionMessage finishOneMsg;
    bool is_load = x2struct::X::loadjson(msg.get()->MessageData, finishOneMsg, false);
    if(is_load) {
      if(finishOneMsg.isFinish) {
        generatePointNode->count_waypoints(poses,fusionMap); //===================== run generate waypoints=================================
      }
    }
  }
    break;
  default: break;

  }

}
