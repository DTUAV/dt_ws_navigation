#include "../include/lidar_slam/icp2d.h"


icp2d::icp2d(): error(0.0), t(Eigen::Vector2f::Zero()), r(Eigen::Matrix2f::Zero()), isInit(false) {

}

icp2d::~icp2d(){

}


void icp2d::init(const std::vector<Eigen::Vector2f> &firstData) {
  lastData = firstData;
  curData = firstData;
  isInit = true;
  std::cout<<"icp2d INIT"<<std::endl;
}


Eigen::Vector2f icp2d::getTranslation() const {
  return t;
}


Eigen::Matrix2f icp2d::getRotation() const {
  return r;
}

double icp2d::getError() const {
  return error;
}
//curData = r*lastData + t; p1为当前数据帧，p2为上一数据帧

void icp2d::runSVD() {
  if(lastData.size() != curData.size()) {
    std::cout<<"icp2d fail====the size of curData != the size of lastData"<<std::endl;
    return;
  }
  //s2: 计算两个点云的中心
  size_t pointNum = curData.size();
  Eigen::Vector2f p1Center, p2Center;
  for(int i = 0; i < pointNum; ++i) {
    p1Center += curData.at(i);
    p2Center += lastData.at(i);
  }
  p1Center /= pointNum;
  p2Center /= pointNum;

  //s3: 计算各个点云与中心的距离
  std::vector<Eigen::Vector2f> q1(pointNum), q2(pointNum);
  for(int i = 0; i < pointNum; ++i) {
    q1.at(i) = curData.at(i) - p1Center;
    q2.at(i) = lastData.at(i) - p2Center;
  }
  //s4: 计算H矩阵
  Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
  for(int i = 0; i < pointNum; ++i) {
    H += q2.at(i) * q1.at(i).transpose();
  }

  //s5: 对H矩阵做SVD分解
  Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2f u = svd.matrixU();
  Eigen::Matrix2f v = svd.matrixV();

  //s6: 计算旋转变换
  r = v * u.transpose();
  t = p1Center - r * p2Center;

  //s7: 计算匹配的偏差
  error = 0;
  for(int i = 0; i < pointNum; ++i) {
    error += (curData.at(i) - r*lastData.at(i) - t).transpose() * (curData.at(i) - r*lastData.at(i) - t);
  }
}


std::pair<double, bool> icp2d::update(const std::vector<Eigen::Vector2f> &dataIn) {
  if(!isInit) {
    std::cout << "icp2d no init == break" << std::endl;
    return std::pair<double, bool>(0.0, false);
  }
  curData = dataIn;
  runSVD();
  lastData = dataIn;
  return std::pair<double, bool>(error, true);
}


std::pair<double, bool> icp2d::update(const std::vector<Eigen::Vector2f> &curDataIn, const std::vector<Eigen::Vector2f> &lastDataIn) {
  lastData = lastDataIn;
  return update(curDataIn);
}


std::pair<double, bool> icp2d::update(const std::vector<Eigen::Vector2f> &curDataIn, const std::vector<Eigen::Vector2f> &lastDataIn, Eigen::Matrix2f &rotationOut, Eigen::Vector2f &translationOut) {
  std::pair<double, bool> ret = update(curDataIn, lastDataIn);
  rotationOut = r;
  translationOut = t;
  return ret;
}










