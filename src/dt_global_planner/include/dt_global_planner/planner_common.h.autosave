#ifndef PLANNER_COMMON_H
#define PLANNER_COMMON_H
#include "iostream"
#include "vector"

//二维的prm路径节点描述
struct graph_node2d
{
  prm_node2d cur_node;//当前对应的栅格点位置
  std::vector<prm_node2d> nodes;
  graph_node(): prm_node2d(0, 0) {}
  graph_node(int x, int y): prm_node2d(x, y) {}
  graph_node(int x, int y, double cost): prm_node2d(x, y, cost) {}
  graph_node(int x, int y, double cost, double dist): prm_node2d(x, y, cost, dist) {}
};

struct prm_node2d {
  prm_node2d(): node2d(0,0), distance(0.0) {}
  prm_node2d(int x, int y): node2d(x, y), distance(0.0) {}
  prm_node2d(int x, int y, double cost): node2d(x, y, cost), distance(0.0) {}
  prm_node2d(int x, int y, double cost, double dist): node2d(x, y, cost), distance(dist) {}
  node2d cur_node;
  double distance;//与主节点的距离
};

struct pos2d
{
  pos2d(int x, int y): pos_x(x), pos_y(y) {}
  pos2d(): pos_x(0), pos_y(0) {}
  int pos_x;//栅格地图中位置x
  int pos_y;//栅格地图中位置y
};

struct robot_body2d
{
  robot_body2d(): body({}){}
  robot_body2d(const std::vector<pos2d> &data): body(data) {}
  robot_body2d(const robot_body2d &data) {
    body = data.body;
  }
  ~robot_body2d() = default;
  robot_body2d& operator =(const robot_body2d &data) {
    if(this != &data) {
      body = data.body;
    }
    return *this;
  }
  std::vector<pos2d> body;

};

struct robot_body3d
{
  std::vector<pos3d> body;
};

struct pos3d
{
  pos3d(int x, int y, int z): pos_x(x), pos_y(y), pos_z(z) {}
  pos3d(): pos_x(0), pos_y(0), pos_z(0) {}
  int pos_x;//栅格地图中位置x
  int pos_y;//栅格地图中位置y
  int pos_z;//栅格地图中位置y
};

struct node2d {
  node2d(): pos_x(0), pos_y(0), cost(0.0) {}
  node2d(int x, int y): pos_x(x), pos_y(y), cost(0.0) {}
  node2d(int x, int y, double cost): pos_x(x), pos_y(y), cost(0.0) {}
  int pos_x; //路径点的位置x
  int pos_y; //路径点的位置y
  double cost;//路径点代价
};

struct node3d {
  node3d(): pos_x(0), pos_y(0), pos_z(0), cost(0.0) {}
  node3d(int x, int y, int z): pos_x(x), pos_y(y), pos_z(z), cost(0.0) {}
  node3d(int x, int y, int z, double cost): pos_x(x), pos_y(y), pos_z(z), cost(cost) {}
  int pos_x; //路径点的位置x
  int pos_y; //路径点的位置y
  int pos_z; //路径点的位置y
  double cost;//路径点代价
};

struct path2d {
  int path_num;//该条路径的路径点数量
  double all_cost;//该条路径的代价
  std::vector<node2d> nodes;//该条路径的所有路径点
};

struct path3d {
  int path_num;//该条路径的路径点数量
  double all_cost;//该条路径的代价
  std::vector<node3d> nodes;//该条路径的所有路径点
};

struct path_node2d {
  bool is_visit;     //该节点是否访问过
  bool is_obstacle;  //该节点是否为障碍物
  int cur_x;         //当前节点在地图的位置x
  int cur_y;         //当前节点在地图的位置y
  int father_x;      //该节点的父节点在地图中位置x
  int father_y;      //该节点的父节点在地图中位置y
  double cost;        //该节点的代价
};

struct path_node3d {
  bool is_visit;     //该节点是否访问过
  bool is_obstacle;  //该节点是否为障碍物
  int cur_x;         //当前节点在地图的位置x
  int cur_y;         //当前节点在地图的位置y
  int cur_z;         //当前节点在地图的位置z
  int father_x;      //该节点的父节点在地图中位置x
  int father_y;      //该节点的父节点在地图中位置y
  int father_z;      //该节点的父节点在地图中位置z
  double cost;        //该节点的代价
};

//比较器，从大到小排序
struct greater1 {
        bool operator()(const path_node2d& a, const path_node2d& b) const {
            return a.cost > b.cost;
        }
};

//

//2d bresenham
void bresenham2d(const int &start_x, const int &start_y, const int &end_x, const int &end_y, std::vector<pos2d> &data) {
   data.clear();
   int dx = std::abs(end_x - start_x);
   int dy = std::abs(end_y - start_y);

   int x = start_x;
   int y = start_y;

   int sx = end_x > start_x ? 1 : -1;
   int sy = end_y > start_y ? 1 : -1;

   if(dx > dy) {
     int e = -dx;
     for(int i = 0; i < dx; ++i) {
       x += sx;
       e += 2 * dy;
       if(e >= 0) {
         y += sy;
         e -= 2 * dx;
       }
       pos2d pos(x,y);
       data.push_back(pos);
     }
   }
   else {
     int e = -dy;
     for(int i = 0; i < dy; ++i) {
       y += sy;
       e += 2 * dx;
       if(e >= 0) {
         x += sx;
         e -= 2 * dy;
       }
       pos2d pos(x, y);
       data.push_back(pos);
     }
   }
}


const double POT_HIGH = 1.0e10; //考虑距离信息的最大的代价，为初始化点和障碍物点

#endif // PLANNER_COMMON_H
