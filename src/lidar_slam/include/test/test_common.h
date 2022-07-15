#ifndef TEST_COMMON_H
#define TEST_COMMON_H
#include "iostream"
#include "utility"
#include "vector"
template<typename point_type, typename rotation_type>
std::pair<std::vector<point_type>, std::vector<point_type>> generateRandomPointPairs(int size, point_type t, rotation_type r, point_type startPoint, point_type movePoint, double noise=0.1 )
{
  std::vector<point_type> lastData(size);
  std::vector<point_type> curData(size);
  for (int i = 0; i < size; ++i)
  {
    point_type noiseMatrix = noise * point_type::Random();
    double noiseValue = drand48();
    point_type lastDataValue =  noiseValue * startPoint + movePoint;
    point_type curDataValue = r * lastDataValue + t + noiseMatrix;
    lastData.at(i) = lastDataValue;
    curData.at(i) = curDataValue;
  }
   std::pair<std::vector<point_type>, std::vector<point_type>> ret(lastData, curData);
   return ret;
}
#endif // TEST_COMMON_H
