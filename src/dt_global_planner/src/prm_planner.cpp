#include "../include/dt_global_planner/prm_planner.h"

bool prm_planner2d::check_valid(const int &start_x, const int &start_y, const int &end_x, const int &end_y) {
  std::vector<pos2d> all_pos;
  //S1: 获取从起点到终点经过的栅格
  bresenham2d(start_x, start_y, end_x, end_y, all_pos);
  if(all_pos.empty()) return false;//没有一个点，说明起点和终点是同一个点
  //S2: 检测所有的栅格点是否是无碰撞的
  for(const auto &val : all_pos) {
    if(get_robot_cost(val.pos_x, val.pos_y) < 0) return false;//只要存在有一个点有碰撞就返回false
  }
  return true;//所有的点都没有碰撞
}

int prm_planner2d::get_gride_dist(const int &x1, const int &y1, const int &x2, const int &y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

//在指定范围内获取随机二维位置
pos2d prm_planner2d::get_range_pose(const int &min_size_x, const int &max_size_x, const int &min_size_y, const int &max_size_y) {
  std::random_device rd;
  std::mt19937 data(rd());
  std::uniform_int_distribution<> dis_x(min_size_x, max_size_x);
  std::uniform_int_distribution<> dis_y(min_size_y, max_size_y);
  pos2d ret;
  ret.pos_x = (int) dis_x(data);
  ret.pos_y = (int) dis_y(data);
  return ret;
}

void prm_planner2d::config_map() {
  cost_map.clear();
  new_cost_map.clear();
  cost_map.resize(map_size_x);
  new_cost_map.resize(map_size_x);
  for(int i = 0; i < map_size_y; ++i) {
    cost_map[i].resize(map_size_y, un_know_cost);
    new_cost_map[i].resize(map_size_y, POT_HIGH);
  }
}

void prm_planner2d::init_robot_body(const robot_body2d &body_data) {

}

bool prm_planner2d::get_path(path2d &path) {

  //从获取nodes中获取一条路径
  if(is_dijkstra) {
    //dijkstra

    //利用map进行查找

  }
  else {
    //A*


  }
}

bool prm_planner2d::update_map() {

  //S1: 确定采样空间的区域（长度和宽度）
  int min_range_x = std::max(0, std::min(start_pos.pos_x, target_pos.pos_x) - offset_x);
  int max_range_x = std::min(map_size_x - 1, std::max(start_pos.pos_x, target_pos.pos_x) + offset_x);

  int min_range_y = std::max(0, std::min(start_pos.pos_y, target_pos.pos_y) - offset_y);
  int max_range_y = std::min(map_size_y - 1, std::max(start_pos.pos_y, target_pos.pos_y) + offset_y);

  //S2: 获取采样点（对于在障碍物的点直接剔除掉）[O(n)]
  std::vector<pos2d> all_sample_pos;

  //将起点和目标点压入采样点容器中
  all_sample_pos.push_back(start_pos);
  all_sample_pos.push_back(target_pos);

  //随机采样指定区域的点
  int num = 0;
  while(num < samples_num) {
    pos2d pos = get_range_pose(min_range_x, max_range_x, min_range_y, max_range_y);
    if(get_robot_cost(pos.pos_x, pos.pos_y) >= 0) {
      all_sample_pos.push_back(pos);
    }
    num++;
  }

  //S3: 按距离进行排序(从小到大排序) [O(nlogn)]
  std::sort(all_sample_pos.begin(), all_sample_pos.end(), [](const pos2d& a, const pos2d& b) {
    return a.pos_x + a.pos_y < b.pos_x + b.pos_y;
  });

  //S4: 对所有采样点按配置的距离进行连线并进行碰撞检测 [最坏O(n*n)]
  //.双指针左右遍历，获取指定区域的所有采样点
  delete[] nodes;
  nodes = new graph_node2d[all_sample_pos.size()];

  for(int i = 0; i < all_sample_pos.size(); ++i) {
    int left = i - 1;
    int right = i + 1;
    bool left_find = false;
    bool right_find = false;
    //保存当前位置的信息
    nodes[i].cur_node.cur_node.pos_x = all_sample_pos[i].pos_x;
    nodes[i].cur_node.cur_node.pos_y = all_sample_pos[i].pos_y;
    nodes[i].cur_node.cur_node.cost = count_cost(cost_map[all_sample_pos[i].pos_x][all_sample_pos[i].pos_y]);

    //保存当前位置的邻居节点的位置及代价信息
    while(!left_find || !right_find) {

      if(!left_find && left >= 0) {
        int dist = get_gride_dist(all_sample_pos[left].pos_x, all_sample_pos[left].pos_y, all_sample_pos[i].pos_x, all_sample_pos[i].pos_y);
        if(dist > search_range) {
          left_find = true;
        }
        else {
          if(check_valid(all_sample_pos[left].pos_x, all_sample_pos[left].pos_y, all_sample_pos[i].pos_x, all_sample_pos[i].pos_y)) {
            prm_node2d left_node;
            left_node.cur_node.pos_x = all_sample_pos[left].pos_x;
            left_node.cur_node.pos_y = all_sample_pos[left].pos_y;
            left_node.cur_node.cost = count_cost(cost_map[left_node.cur_node.pos_x][left_node.cur_node.pos_y]) + dist_cost_scale * dist;//代价后加上与父节点的距离
            left_node.distance = dist;
            nodes[i].nodes.push_back(left_node);
          }
          left--;
        }
      }
      else {
        left_find = true;
      }

      if(!right_find && right < all_sample_pos.size()) {
        int dist = get_gride_dist(all_sample_pos[right].pos_x, all_sample_pos[right].pos_y, all_sample_pos[i].pos_x, all_sample_pos[i].pos_y);
        if(dist > search_range) {
          right_find = true;
        }
        else {
          if(check_valid(all_sample_pos[right].pos_x, all_sample_pos[right].pos_y, all_sample_pos[i].pos_x, all_sample_pos[i].pos_y)) {
            prm_node2d right_node;
            right_node.cur_node.pos_x = all_sample_pos[right].pos_x;
            right_node.cur_node.pos_y = all_sample_pos[right].pos_y;
            right_node.cur_node.cost = count_cost(cost_map[right_node.cur_node.pos_x][right_node.cur_node.pos_y]) + dist_cost_scale * dist;//代价后加上与父节点的距离
            right_node.distance = dist;
            nodes[i].nodes.push_back(right_node);
          }
          right++;
        }
      }
      else {
        right_find = true;
      }
    }
  }

  //S5: 利用最终的采样点构建路径规划的代价地图
  new_cost_map.clear();
  new_cost_map.resize(map_size_x);
  for(int i = 0; i < map_size_y; ++i) {
    new_cost_map[i].resize(map_size_y, POT_HIGH);
  }

  //更新采样点的代价, 用于显示采样点
  for(int i = 0; i < all_sample_pos.size(); ++i) {
    new_cost_map[all_sample_pos[i].pos_x][all_sample_pos[i].pos_y] = std::min(nodes[i].cur_node.cur_node.cost, new_cost_map[all_sample_pos[i].pos_x][all_sample_pos[i].pos_y]);
   // for(const auto &val : nodes[i].nodes) {
   //   new_cost_map[val.cur_node.pos_x][val.cur_node.pos_y] = std::min(new_cost_map[val.cur_node.pos_x][val.cur_node.pos_y], val.cur_node.cost);
   // }
  }

}

double prm_planner2d::count_cost(int raw_cost) {
  //根据选择的搜索算法配置代价地图
    if(raw_cost >= lethal_cost || ((!is_plan_unknow)&&raw_cost == un_know_cost))
      return POI_HIGH;
    double cost = raw_cost * cost_scale + neutral_cost; //一次函数缩放
    if(cost >= lethal_cost)
      cost = lethal_cost - 1;
    return cost;
}

double prm_planner2d::count_new_cost(const int &raw_cost, const int &x, const int &y) {
  if(is_dijkstra) return count_cost(raw_cost);
  double cost = count_cost(raw_cost) + dist_cost_scale * (abs(x - target_pos.pos_x) + abs(y - target_pos.y));
  return cost;
}

double prm_planner2d::get_robot_cost(const int &x, const int &y) {

}

double prm_planner2d::get_robot_cost(const int &x, const int &y, const robot_body2d &body) {

}

prm_planner2d::~prm_planner2d() {

}
