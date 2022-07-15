#include "../include/lidar_slam/icp2d.h"
#include "../include/test/test_common.h"
int main(int argv, char** argc) {
  Eigen::Vector2f t = Eigen::Vector2f::Zero();
  t << 3, 1;
  std::cout << t << std::endl;
  Eigen::Matrix2f r = Eigen::Matrix2f::Zero();
  r<< 0.5, 0.86, -0.86, 0.5;
  Eigen::Vector2f startPoint(50, 10);
  Eigen::Vector2f movePoint(4, 7);

  std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f> > dataSet = generateRandomPointPairs<Eigen::Vector2f, Eigen::Matrix2f>(100, t, r, startPoint, movePoint);
  //for(auto val : dataSet.first) {
  //  std::cout << val << std::endl;
  //}
  icp2d icpNode;
  icpNode.init(dataSet.first);
  icpNode.update(dataSet.second);
  double error = icpNode.getError();
  std::cout << error << std::endl;
  Eigen::Vector2f resultT = icpNode.getTranslation();
  Eigen::Matrix2f resultR = icpNode.getRotation();
  std::cout << resultT << std::endl;
  std::cout << resultR << std::endl;

}
