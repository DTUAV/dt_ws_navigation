#ifndef ICP2D_H
#define ICP2D_H
#include <utility>
#include <list>
#include <vector>
#include "eigen3/Eigen/Eigen"
#include "iostream"
class icp2d
{
public:
  icp2d();
  ~icp2d();
  void init(const std::vector<Eigen::Vector2f>& firstData); //初始化第一帧点云
  std::pair<double, bool> update(const std::vector<Eigen::Vector2f> &dataIn);//输入当前帧的点云, 进行点云匹配，返回匹配的误差和是否完成匹配
  std::pair<double, bool> update(const std::vector<Eigen::Vector2f> &curDataIn, const std::vector<Eigen::Vector2f> &lastDataIn);//输入当前帧的点云和上一帧点云, 进行点云匹配，返回匹配的误差和是否完成匹配
  std::pair<double, bool> update(const std::vector<Eigen::Vector2f> &curDataIn, const std::vector<Eigen::Vector2f> &lastDataIn, Eigen::Matrix2f &rotationOut, Eigen::Vector2f &translationOut);//输入当前帧的点云, 上一帧点云, 旋转的引用，平移的引用进行点云匹配，返回匹配的误差和是否完成匹配
  Eigen::Vector2f getTranslation() const;
  Eigen::Matrix2f getRotation() const;
  double getError() const;

private:
  void runSVD();
  std::vector<Eigen::Vector2f> lastData; //上一帧点云数据
  std::vector<Eigen::Vector2f> curData; //当前一帧点云数据
  double error;       //点云匹配的误差
  Eigen::Vector2f t;       //点云的平移
  Eigen::Matrix2f r;    //点云的旋转
  bool isInit;

};

#endif // ICP2D_H
