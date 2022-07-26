#ifndef BASE_PLANNER_H
#define BASE_PLANNER_H
#include "vector"
#include "planner_common.h"
#define DEBUG

//二维路径规划算法基类
class base_planner2d {
public:
  base_planner2d(int size_x, int size_y, long iter_num, bool plan_unknow):
    map_size_x(size_x), map_size_y(size_y), max_iter_num(iter_num), un_know_cost(255), lethal_cost(253), neutral_cost(50), is_get_map(false), is_get_path(false), is_init_body(false), is_plan_unknow(plan_unknow) {
  }
  base_planner2d(int size_x, int size_y, long iter_num, unsigned char un_know, unsigned char lethal, unsigned char neutral, bool plan_unknow):
    map_size_x(size_x), map_size_y(size_y), max_iter_num(iter_num), un_know_cost(un_know), lethal_cost(lethal), neutral_cost(neutral), is_get_map(false), is_get_path(false), is_init_body(false), is_plan_unknow(plan_unknow){
  }
  virtual ~base_planner2d();

  //初始化地图
  virtual void config_map();

  //获取规划的路径
  virtual bool get_path(path2d &path);

  //主要是更新考虑距离信息的新代价地图
  virtual bool update_map();

  //路径规划过程代价计算函数
  virtual double count_cost(int raw_cost);

  //初始底盘位置数据
  virtual void init_robot_body(const robot_body2d &body);

  //获取机器人在指定点的碰撞代价, body为机器人底盘在栅格地图的位置
  virtual double get_robot_cost(const int &x, const int &y, const robot_body2d &body);
  virtual double get_robot_cost(const int &x, const int &y);

  //这里的起点和目标点都是对应栅格地图的点
  bool set_start_end_pos(const pos2d &start, pos2d &target) {
    if(!is_get_map) {
#ifdef DEBUG
      std::cerr.flush();
      std::cerr << "base_planner2d: set_start_end_pos: is_get_map != true, no cost map has been get" << std::endl;
#endif
      return false;
    }
    //检查设置的目标点是否为障碍物点并进行平移
    int goal_x = target.pos_x;
    int goal_y = target.pos_y;

    int start_x = start.pos_x;
    int start_y = start.pos_y;

    int diff_x = goal_x - start_x;
    int diff_y = goal_y - start_y;

    int target_x = goal_x;
    int target_y = goal_y;

    bool done = false;
    double scale = 1.0;
    double d_scale = 0.01;

    while(!done) {
      if(scale < 0) {
        target_x = start_x;
        target_y = start_y;
#ifdef DEBUG
        std::cerr.flush();
        std::cerr << "base_planner2d: set_start_end_pos: the target is in obstacle area and not find valid target pos" << std::endl;
#endif
        break;
      }
      target_x = start_x + int(scale * diff_x);
      target_y = start_y + int(scale * diff_y);
      double cost = get_robot_cost(target_x, target_y); //如果cost小于0，则认为此点为障碍物点
      if(cost >= 0)
        done = true;
      scale -= d_scale;
    }
    start_pos.pos_x = start_x;
    start_pos.pos_y = start_y;
    target_pos.pos_x = target_x;
    target_pos.pos_y = target_y;
    target.pos_x = target_x;//更新实际的目标点x
    target.pos_y = target_y;//更新实际的目标点y
    return done;
  }

  //将代价地图更新本地的代价地图, 每次路径规划前需要获取最新的代价地图
  bool get_env_cost_map(unsigned char *map, int size_x, int size_y) {
    if(size_x != map_size_x || size_y != map_size_y) {
#ifdef DEBUG
      std::cerr.flush();
      std::cerr << "base_planner2d: get_env_cost_map: the map_size_x or map_size_y != size_x or size_y, resize the cost_map size" << std::endl;
#endif
      map_size_x = size_x;
      map_size_y = size_y;
      config_map();
    }
    for(size_t i = 0; i < size_x; ++i) {
      for(size_t j = 0; j < size_y; ++j) {
        cost_map[i][j] = map[i * size_x + j];
      }
    }
    is_get_map = true;
  }

  //设置未知区域的代价
  void set_un_know_cost(unsigned char cost) {
    un_know_cost = cost;
  }

  //设置障碍物点的代价
  void set_lethal_cost(unsigned char cost) {
    lethal_cost = cost;
  }

  //设置中性点的代价
  void set_neutral_cost(unsigned char cost) {
    neutral_cost = cost;
  }

  //获取地图的宽度x
  int get_map_size_x() const {
    return map_size_x;
  }
  //获取地图的长度y
  int get_map_size_y() const {
    return map_size_y;
  }

  //获取目标点
  pos2d get_target_pos() const {
    return target_pos;
  }

  //获取起点
  pos2d get_start_pos() const {
    return start_pos;
  }

  //获取代价地图
  const std::vector<std::vector<int> >& get_cost_map() {
    return cost_map;
  }

  //获取考虑距离信息后的代价地图
  const std::vector<std::vector<double> >& get_new_cost_map() {
    return new_cost_map;
  }

protected:
  int map_size_x; //地图的宽度
  int map_size_y; //地图的长度
  long max_iter_num;//最大迭代次数
  unsigned char un_know_cost;//信息未知的代价
  unsigned char lethal_cost;//致命的代价，规划的路径要尽可以远离
  unsigned char neutral_cost;//中性代价，规划的路径要尽可能靠近
  pos2d start_pos;//路径的起点，栅格地图中的点
  pos2d target_pos;//路径的目标点，栅格地图中的点
  bool is_get_map;//是否获取到原始的代价地图
  bool is_get_path;//是否找到路径
  bool is_plan_unknow;//是否可以在未知环境中规划路径
  bool is_init_body;//是否已经配置了机器人底盘位置数据
  robot_body2d body;//机器人底盘在栅格地图中的位置
  std::vector<std::vector<int> > cost_map;//代价的容器(没有考虑距离信息)
  std::vector<std::vector<double> > new_cost_map;//考虑距离信息后的代价地图
};

//三维路径规划算法的基类
class base_planner3d : public base_planner2d
{
public:
  base_planner3d(int size_x, int size_y);
  virtual bool get_path3d(path3d &path);
  virtual void config_map();
  virtual bool get_path(path2d &path);
  virtual bool update_map();
  virtual double count_cost(int raw_cost);
  virtual void init_robot_body3d(const robot_body3d& body3d);
  virtual void init_robot_body(const robot_body2d& body);
  virtual double get_robot_cost(const int &x, const int &y);
  virtual double get_robot_cost3d(const int &x, const int &y);
  virtual double get_robot_cost(const int &x, const int &y, const robot_body2d& body);
  virtual double get_robot_cost3d(const int &x, const int &y, const robot_body3d& body3d);

  bool set_start_end_pos3d(const pos3d &start, pos3d &target) {
    if(!is_get_map) {
#ifdef DEBUG
      std::cerr.flush();
      std::cerr << "base_planner3d: set_start_end_pos3d: is_get_map != true, no cost map has been get" << std::endl;
#endif
      return false;
    }
    int goal_x = target.pos_x;
    int goal_y = target.pos_y;
    int goal_z = target.pos_z;

    int start_x = start.pos_x;
    int start_y = start.pos_y;
    int start_z = start.pos_z;

    int diff_x = goal_x - start_x;
    int diff_y = goal_y - start_y;
    int diff_z = goal_z - start_z;

    int target_x = goal_x;
    int target_y = goal_y;
    int target_z = goal_z;

    bool done = false;
    double scale = 1.0;
    double d_scale = 0.01;
    while(!done) {
      if(scale < 0) {
        target_x = start_x;
        target_y = start_y;
        target_z = start_z;
#ifdef DEBUG
        std::cerr.flush();
        std::cerr << "base_planner3d: set_start_end_pos3d: the target is in obstacle area and not find valid target pos" << std::endl;
#endif
        break;
      }
      target_x = start_x + int(scale * diff_x);
      target_y = start_y + int(scale * diff_y);
      target_z = start_z + int(scale * diff_z);
      double cost = get_robot_cost3d(target_x, target_y, target_z); //如果cost小于0，则认为此点为障碍物点
      if(cost >= 0)
        done = true;
      scale -= d_scale;
    }
    start_pos.pos_x = start_x;
    start_pos.pos_y = start_y;
    target_pos.pos_x = target_x;
    target_pos.pos_y = target_y;

    start_pos3d.pos_x = start_x;
    start_pos3d.pos_y = start_y;
    start_pos3d.pos_z = start_z;

    target_pos3d.pos_x = target_x;
    target_pos3d.pos_y = target_y;
    target_pos3d.pos_z = target_z;

    target.pos_x = target_x;//更新实际的目标点x
    target.pos_y = target_y;//更新实际的目标点y
    target.pos_z = target_z;
    return done;
  }

  //重写更新地图的函数
  bool get_env_cost_map(unsigned char *map, int size_x, int size_y) override {

  }

  //三维地图的更新函数
  bool get_env_cost_map3d(unsigned char **map, int size_x, int size_y, int size_z) {
    //map[size_x * size_y][size_z]
    if(size_x != map_size_x || size_y != map_size_y || size_z != map_size_z) {
#ifdef DEBUG
      std::cerr.flush();
      std::cerr << "base_planner3d: get_env_cost_map3d: the map_size_x or map_size_y or map_size_z != size_x or size_y or size_z, resize the cost_map size" << std::endl;
#endif
      map_size_x = size_x;
      map_size_y = size_y;
      map_size_z = size_z;
      config_map();
    }
    for(size_t i = 0; i < size_x * size_y; ++i) {
      for(size_t j = 0; j < size_z; ++j) {
        cost_map[i][j] = map[i][j];
      }
    }
    is_get_map = true;

  }


protected:
  int map_size_z; //地图的高度
  pos3d start_pos3d;//路径的起点3d
  pos3d target_pos3d;//路径的终点3d
  robot_body3d body3d;//机器人机身三维坐标点
};



#endif // BASE_PLANNER_H
