#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H
#include "base_planner.h"
#include "planner_common.h"
#include <memory>
#include <algorithm>

class astar_planner2d : public base_planner2d
{
public:
  /*
   * size_x: 地图的大小x
   * size_y: 地图的大小y
   * iter_num: 路径规划过程中迭代的次数
   * plan_unknow: 是否在未知的区域导航
   * scale: 原始代价的缩放比例因子
   * dist_scale: 距离的代价缩放比例因子，计算当前位置和目标点位置的距离的代价
   */
  astar_planner2d(int size_x, int size_y, long iter_num, bool plan_unknow, double scale, double dist_scale): base_planner2d(size_x, size_y, iter_num, plan_unknow), cost_scale(scale), distance_scale(dist_scale) {
    config_map();
    all_nodes.resize(size_x * size_y);
    reset_all_nodes();
  }
  virtual void config_map();
  virtual void init_robot_body(const robot_body2d &body_data);
  virtual bool get_path(path2d &path);
  virtual bool update_map();
  virtual double count_cost(int raw_cost);
  virtual double get_robot_cost(const int &x, const int &y);
  virtual double get_robot_cost(const int &x, const int &y, const robot_body2d &body);
  virtual ~astar_planner2d();
  double count_new_cost(const int& raw_cost, const int &x, const int &y);
  bool make_path(unsigned char *map, const int &size_x, const int &size_y, const pos2d &start, pos2d &target);
  void reset_all_nodes();
  bool add_data(const double &cost, const int &farther_x, const int &farther_y);
private:
  double cost_scale;//原始代价的缩放比例因子
  double distance_scale;//距离的代价缩放比例因子，计算当前位置和目标点位置的距离的代价
  path_node2d data_need_vist;//下一个需要访问的路径点
  std::vector<path_node2d> all_nodes; //深度搜索得到的所有路径点
};




#endif // ASTAR_PLANNER_H
