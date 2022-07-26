#ifndef PLANNER_COMMON_H
#define PLANNER_COMMON_H
#include "iostream"
#include "vector"

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
  int pos_x; //路径点的位置x
  int pos_y; //路径点的位置y
  double cost;//路径点代价
};

struct node3d {
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


const double POT_HIGH = 1.0e10; //考虑距离信息的最大的代价，为初始化点和障碍物点

#endif // PLANNER_COMMON_H
