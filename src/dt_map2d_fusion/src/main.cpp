#include "../include/dt_map2d_fusion/data_source.h"

int main(int argv, char** argc) {
  ros::init(argv, argc, "dt_map2d_fusion_node");
  dt_map2d_fusion::data_source node;
  ros::spin();
}
