#include "../include/dt_map2d_fusion/data_source.h"

using namespace dt_map2d_fusion;

int main(int argc, char** argv) {

  ros::init(argc, argv, "test_data_source_node");
  data_source data_source_node;
  ros::spin();
}
