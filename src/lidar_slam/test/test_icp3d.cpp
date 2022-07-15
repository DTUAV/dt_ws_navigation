#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include "../include/lidar_slam/icp3d.h"
int main(int argc, char** argv) {
    Eigen::AngleAxisf angle_axis(M_PI / 3, Eigen::Vector3f(0, 0, 1));
    Eigen::Matrix3f R_12_ = angle_axis.matrix();
    Eigen::Vector3f t_12_(1, 2, 3);
    std::cout << "except R_12:\n" << R_12_ << std::endl;
    std::cout << "except t_12:\n" << t_12_.transpose() << std::endl;

    std::vector<Eigen::Vector3f> p1, p2;
    Eigen::Vector3f point;

    std::ifstream fin("dataSet.txt");
    std::string line;
    while (getline(fin, line)) {
        std::stringstream ss(line);
        ss >> point.x();
        ss >> point.y();
        ss >> point.z();
        p2.push_back(point);
        p1.push_back(R_12_ * point + t_12_);
    }
    fin.close();

    icp3d icpNode;
    icpNode.init(p2);
    icpNode.update(p1);
    double error = icpNode.getError();
    std::cout << error << std::endl;
    Eigen::Vector3f resultT = icpNode.getTranslation();
    Eigen::Matrix3f resultR = icpNode.getRotation();
    std::cout << resultT << std::endl;
    std::cout << resultR << std::endl;
    return 0;
}

