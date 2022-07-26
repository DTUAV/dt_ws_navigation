#include "../include/dt_global_planner/astar_planner.h"

//析构函数
astar_planner2d::~astar_planner2d() {

}

//初始化机器人底盘的位置数据
void astar_planner2d::init_robot_body(const robot_body2d &body_data) {
  body = body_data;
  is_init_body = true;
}

void astar_planner2d::config_map() {
  cost_map.clear();
  new_cost_map.clear();
  cost_map.resize(map_size_x);
  new_cost_map.resize(map_size_x);
  for(int i = 0; i < map_size_y; ++i) {
    cost_map[i].resize(map_size_y, un_know_cost);
    new_cost_map[i].resize(map_size_y, POT_HIGH);
  }
}


//获取搜索到的路径
bool astar_planner2d::get_path(path2d &path) {
  if(!is_get_path) {
#ifdef DEBUG
    std::cerr.flush();
    std::cerr << "astart_planner2d: get_path: is_get_path == false, no path" << std::endl;
#endif
    return false;
  }
  int target_index = target_pos.pos_x * target_pos.pos_y;
  int start_index = start_pos.pos_x * start_pos.pos_y;

  double all_cost = 0;
  int nodes_num = 0;
  path.nodes.clear();

  while(target_index != start_index) {
    node2d node;
    node.pos_x = target_index / map_size_x;
    node.pos_y = target_index % map_size_x;
    node.cost = all_nodes.at(target_index).cost;
    all_cost += node.cost;
    nodes_num++;
    path.nodes.push_back(node);
    target_index = all_nodes.at(target_index).father_x * all_nodes.at(target_index).father_y;
  }

  nodes_num++;
  node2d start_node;
  start_node.cost = all_nodes.at(target_index).cost;
  start_node.pos_x = start_pos.pos_x;
  start_node.pos_y = start_pos.pos_y;
  path.nodes.push_back(start_node);
  all_cost += start_node.cost;
  path.all_cost = all_cost;
  path.path_num = nodes_num;

  std::reverse(path.nodes.begin(), path.nodes.end());//从起点到目标点
  return true;
}

//更新考虑距离信息、目标位置信息、代价地图信息的地图
bool astar_planner2d::update_map() {
  if(!is_get_map) {
#ifdef DEBUG
    std::cerr.flush();
    std::cerr << "dijkstra_planner2d: update_map: is_get_map == false, no update map" << std::endl;
#endif
    return false;
  }
  //reset_all_nodes();
  long cur_iter = 0;
  int start_index = start_pos.pos_x * start_pos.pos_y;
  data_need_vist = all_nodes.at(start_index);
  int target_index = target_pos.pos_x * target_pos.pos_y;
  while(cur_iter <= max_iter_num) {

    int index = data_need_vist.cur_x * data_need_vist.cur_y;
    all_nodes.at(index).is_visit = true;
    all_nodes.at(index).cost = count_cost(cost_map.at(data_need_vist.cur_x).at(data_need_vist.cur_y));

    if(all_nodes.at(index).cost == POT_HIGH)
      all_nodes.at(index).is_obstacle = true;
    else
      all_nodes.at(index).is_obstacle = false;

    if(index == target_index) {
      for(int i = 0; i < all_nodes.size(); ++i) {
        new_cost_map[i / 2][i % 2] = all_nodes.at(i).cost;
      }
      return true;
    }
    if(!add_data(data_need_vist.cur_x, data_need_vist.cur_y, cost_map.at(data_need_vist.cur_x).at(data_need_vist.cur_y)))
      return false;
  }
}

bool astar_planner2d::add_data(const int &farther_x, const int &farther_y, const double &farther_cost) {

  //左边的点
  int left_pos_x = std::max(0, std::min(farther_x - 1, map_size_x));
  int left_pos_y = std::max(0, std::min(farther_y, map_size_y));
  double left_pos_cost = count_new_cost(cost_map.at(left_pos_x).at(left_pos_y), left_pos_x, left_pos_y);

  //右边的点
  int right_pos_x = std::max(0, std::min(farther_x + 1, map_size_x));
  int right_pos_y = left_pos_y;
  double right_pos_cost = count_new_cost(cost_map.at(right_pos_x).at(right_pos_y), right_pos_x, right_pos_y);

  //上面的点
  int up_pos_x = std::max(0, std::min(farther_x, map_size_x));
  int up_pos_y = std::max(0, std::min(farther_y - 1, map_size_y));
  double up_pos_cost = count_new_cost(cost_map.at(up_pos_x).at(up_pos_y), up_pos_x, up_pos_y);

  //下面的点
  int down_pos_x = up_pos_x;
  int down_pos_y = std::max(0, std::min(farther_y + 1, map_size_y));
  double down_pos_cost = count_new_cost(cost_map.at(down_pos_x).at(down_pos_y), down_pos_x, down_pos_y);

  int min_cost_pos_x = left_pos_x;
  int min_cost_pos_y = left_pos_y;
  unsigned int min_cost = left_pos_cost;

  if(right_pos_cost < min_cost) {
    min_cost = right_pos_cost;
    min_cost_pos_x = right_pos_x;
    min_cost_pos_y = right_pos_y;
  }

  if(up_pos_cost < min_cost) {
    min_cost = up_pos_cost;
    min_cost_pos_x = up_pos_x;
    min_cost_pos_y = up_pos_y;
  }

  if(down_pos_cost < min_cost) {
    min_cost = down_pos_cost;
    min_cost_pos_x = down_pos_x;
    min_cost_pos_y = down_pos_y;
  }

  if(min_cost_pos_x == farther_x && min_cost_pos_y == farther_y) {
#ifdef DEBUG
    std::cerr.flush();
    std::cerr << "astar_planner2d: add_data: min_cost_pos_x == farther_x && min_cost_pos_y == farther_y" << std::endl;
#endif
    return false;
  }


  //障碍物情况
  path_node2d data = all_nodes.at(min_cost_pos_x * min_cost_pos_y);
  if(cost_map.at(min_cost_pos_x).at(min_cost_pos_y) >= lethal_cost || ((!is_plan_unknow) && cost_map.at(min_cost_pos_x).at(min_cost_pos_y) == un_know_cost)) {
    data.cost = POT_HIGH;
    data.is_obstacle = true;
    data.is_visit = true;
    all_nodes.at(min_cost_pos_x * min_cost_pos_y) = data;
#ifdef DEBUG
    std::cerr.flush();
    std::cerr << "astar_planner2d: add_data: min_cost_pos_x && min_cost_pos_y is obstacle" << std::endl;
#endif
    return false;
  }

  //如果之前的代价比这次大，说明这次优，选择这一次
  if(data.is_visit) {
    if(min_cost + farther_cost + 1 < data.cost) {
      data.cost = min_cost + farther_cost + 1;//1是距离
      data.father_x = farther_x;
      data.father_y = farther_y;
      all_nodes.at(min_cost_pos_x * min_cost_pos_y) = data;
    }
  }

  else {//这个是没有访问过的，更新信息并加入需要访问的队列
    data.is_visit = true;
    data.father_x = farther_x;
    data.father_y = farther_y;
    data.is_obstacle = false;
    data.cost = min_cost + farther_cost  + 1;//1是距离
    all_nodes.at(min_cost_pos_x * min_cost_pos_y) = data;
  }
  data_need_vist = data;
  return true;
}

//计算考虑距离信息和目标点信息的代价
double astar_planner2d::count_cost(int raw_cost) {
  if(raw_cost >= lethal_cost || ((!is_plan_unknow)&&raw_cost == un_know_cost))
    return POI_HIGH;
  double cost = raw_cost * cost_scale + neutral_cost; //一次函数缩放
  if(cost >= lethal_cost)
    cost = lethal_cost - 1;
  return cost;
}

double astar_planner2d::count_new_cost(const int &raw_cost, const int &x, const int &y) {
  double cost = count_cost(raw_cost) + distance_scale * (abs(x - target_pos.pos_x) + abs(y - target_pos.y));
  return cost;
}

//获取指定位置的机器人底盘的代价，用于判断该位置是否可行
double astar_planner2d::get_robot_cost(const int &x, const int &y) {
  if(!is_init_body) {
#ifdef DEBUG
    std::cerr.flush();
    std::cerr << "digkstra_planner2d: get_robot_cost: is_init_body != true" << std::endl;
#endif
    return -1.0;
  }
  return get_robot_cost(x, y, body);
}

double astar_planner2d::get_robot_cost(const int &x, const int &y, const robot_body2d &body) {
  double cost = 0.0;
  for(const auto &val: body.body) {
    int raw_cost = cost_map.at(val.pos_x).at(val.pos_y);
    if(raw_cost >= lethal_cost - 5 || ((!is_plan_unknow)&&raw_cost == un_know_cost)) //5是远离障碍物的点距离，可以配置,需要按实际情况配置
      return -1.0;
    else
      cost += cost * cost_scale;//需要按实际情况配置
  }
  return cost;
}

//规划路径的主要函数，接受代价地图及大小、初始位置和目标位置，生成可行路径
bool astar_planner2d::make_path(unsigned char *map, const int &size_x, const int &size_y, const pos2d &start, pos2d &target) {
  //S1: 将代价地图更新本地的代价地图, 每次路径规划前需要获取最新的代价地图
  get_env_cost_map(map, size_x, size_y);
  //S2: 设置起点和目标点
  if(!set_start_end_pos(start, target))
    return false;
  //S4: 更新考虑距离信息后的地图
  if(!update_map())
    return false;
  return true;
}

//重置容器的信息
void astar_planner2d::reset_all_nodes() {
  int index = 0;
  for(auto &val : all_nodes){
    val.cur_x = index / size_x;
    val.cur_y = index % size_x;
    val.cost = POT_HIGH;
    val.father_x = 0;
    val.father_y = 0;
    val.is_obstacle = false;
    val.is_visit = false;
    index++;
  }
}
