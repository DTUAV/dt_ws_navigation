#ifndef COMMON_H
#define COMMON_H
#include "vector"

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
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;
} waypoint;

#endif // COMMON_H
