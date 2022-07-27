#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H
#include "base_planner.h"
#include "planner_common.h"
#include <algorithm>
//随机路标图
class prm_planner2d : public base_planner2d
{
public:
  prm_planner2d(int size_x, int size_y, long iter_num, bool plan_unknow, int rg_offset_x, int rg_offset_y, int samples_number, double sr_range, double cs_scale, double ds_cost_scale, bool dijkstra):
    base_planner2d(size_x, size_y, iter_num, plan_unknow), offset_x(rg_offset_x), offset_y(rg_offset_y), samples_num(samples_number), search_range(sr_range), cost_scale(cs_scale), dist_cost_scale(ds_cost_scale), is_dijkstra(dijkstra) {
    config_map();
   }
  virtual void config_map();
  virtual void init_robot_body(const robot_body2d &body_data);
  virtual bool get_path(path2d &path);
  virtual bool update_map();
  virtual double count_cost(int raw_cost);
  virtual double get_robot_cost(const int &x, const int &y);
  virtual double get_robot_cost(const int &x, const int &y, const robot_body2d &body);
  virtual ~prm_planner2d();
  double count_new_cost(const int& raw_cost, const int &x, const int &y);

private:
  int offset_x;//采样区域在原有区域上的偏移x
  int offset_y;//采样区域在原有区域上的偏移y
  int samples_num; //采样的粒子数目
  double search_range;//每个粒子最大探测范围
  double cost_scale;//针对障碍物的代价缩放比例因子
  double dist_cost_scale;//针对A*算法的距离缩放比例因子
  bool is_dijkstra;//是否选择dijkstra算法，true: dijkstra, false: A*
  graph_node2d* nodes;//采样后各个粒子的位置及邻居位置
  pos2d get_range_pose(const int &min_size_x, const int &max_size_x, const int &min_size_y, const int &max_size_y);
  int get_gride_dist(const int &x1, const int &y1, const int &x2, const int &y2);
  bool check_valid(const int &start_x, const int &start_y, const int &end_x, const int &end_y);
};

#endif // PRM_PLANNER_H
