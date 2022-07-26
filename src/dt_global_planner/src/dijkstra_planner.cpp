#include "../include/dt_global_planner/dijkstra_planner.h"

void dijkstra_planner2d::config_map() {
  cost_map.clear();
  new_cost_map.clear();
  cost_map.resize(map_size_x);
  new_cost_map.resize(map_size_x);
  for(int i = 0; i < map_size_y; ++i) {
    cost_map[i].resize(map_size_y, un_know_cost);
    new_cost_map[i].resize(map_size_y, POT_HIGH);
  }
}

bool dijkstra_planner2d::get_path(path2d &path) {
  if(!is_get_path) {
#ifdef DEBUG
    std::cerr.flush();
    std::cerr << "dijkstra_planner2d: get_path: is_get_path == false, no path" << std::endl;
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

bool dijkstra_planner2d::update_map() {
  if(!is_get_map) {
#ifdef DEBUG
    std::cerr.flush();
    std::cerr << "dijkstra_planner2d: update_map: is_get_map == false, no update map" << std::endl;
#endif
    return false;
  }
  data_need_vist.clear();
  //reset_all_nodes();
  long cur_iter = 0;
  int start_index = start_pos.pos_x * start_pos.pos_y;
  data_need_vist.push_back(all_nodes.at(start_index));
  int target_index = target_pos.pos_x * target_pos.pos_y;
  while(cur_iter <= max_iter_num && !data_need_vist.empty()) {

    path_node2d top = data_need_vist.front();
    std::pop_heap(data_need_vist.begin(), data_need_vist.end(), greater1);//将第0个元素和最后一个元素交换位置，并对前n-1个元素进行从小到大排序。也就是第0个元素的代价是最小的
    data_need_vist.pop_back();//最小代价的元素已经保存在top了，这里将这个元素从容器中弹出

    int index = top.cur_x * top.cur_y;
    all_nodes.at(index).is_visit = true;
    all_nodes.at(index).cost = count_cost(cost_map.at(top.cur_x).at(top.cur_y));

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
    //检查右边的点
    add_data(top.cur_x + 1, top.cur_y, all_nodes.at(index).cost, top.cur_x, top.cur_y);
    //检查左边的点
    add_data(top.cur_x - 1, top.cur_y, all_nodes.at(index).cost, top.cur_x, top.cur_y);
    //检查上面的点
    add_data(top.cur_x, top.cur_y - 1, all_nodes.at(index).cost, top.cur_x, top.cur_y);
    //检查下面的点
    add_data(top.cur_x, top.cur_y + 1, all_nodes.at(index).cost, top.cur_x, top.cur_y);
    cur_iter++;
  }
  return false;
}

void dijkstra_planner2d::add_data(const int &x, const int &y, const double &cost, const int &farther_x, const int &farther_y) {
  if(x * y < 0 || x * y >= map_size_x * map_size_y)//超出地图边界情况
    return;
  //障碍物情况
  path_node2d data = all_nodes.at(x * y);
  if(cost_map.at(x).at(y) >= lethal_cost || ((!is_plan_unknow) && cost_map.at(x).at(y) == un_know_cost)) {
    data.cost = POT_HIGH;
    data.is_obstacle = true;
    data.is_visit = true;
    all_nodes.at(x * y) = data;
    return;
  }

  //如果之前的代价比这次大，说明这次优，选择这一次
  if(data.is_visit) {
    if(cost + 1 < data.cost) {
      data.cost = cost + 1;//1是距离
      data.father_x = farther_x;
      data.father_y = farther_y;
      all_nodes.at(x * y) = data;
    }
  }

  else {//这个是没有访问过的，更新信息并加入需要访问的队列
    data.is_visit = true;
    data.father_x = farther_x;
    data.father_y = farther_y;
    data.is_obstacle = false;
    data.cost = cost + 1;//1是距离
    all_nodes.at(x * y) = data;
    data_need_vist.push_back(data);
  }
}

double dijkstra_planner2d::count_cost(int raw_cost) {
  if(raw_cost >= lethal_cost || ((!is_plan_unknow)&&raw_cost == un_know_cost))
    return POI_HIGH;
  double cost = raw_cost * cost_scale + neutral_cost; //一次函数缩放
  if(cost >= lethal_cost)
    cost = lethal_cost - 1;
  return cost;
}

void dijkstra_planner2d::reset_all_nodes() {
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

void dijkstra_planner2d::init_robot_body(const robot_body2d &body_data) {
  body = body_data;
  is_init_body = true;
}

double dijkstra_planner2d::get_robot_cost(const int &x, const int &y) {
  if(!is_init_body) {
#ifdef DEBUG
    std::cerr.flush();
    std::cerr << "digkstra_planner2d: get_robot_cost: is_init_body != true" << std::endl;
#endif
    return -1.0;
  }
  return get_robot_cost(x, y, body);
}

double dijkstra_planner2d::get_robot_cost(const int &x, const int &y, const robot_body2d &body) {
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

bool dijkstra_planner2d::make_path(unsigned char *map, const int &size_x, const int &size_y, const pos2d &start, pos2d &target) {

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
