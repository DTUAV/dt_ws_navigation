/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
//

#ifndef _NAVFN_H
#define _NAVFN_H

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// cost defs
#define COST_UNKNOWN_ROS 255		// 255 is unknown cost// 未知区域的代价是255
#define COST_OBS 254		// 254 for forbidden regions// 禁止区域的代价是254
#define COST_OBS_ROS 253	// ROS values of 253 are obstacles  //障碍物的代价是253

// navfn cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
// Incoming costmap cost values are in the range 0 to 252.
// With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
// ensure the input values are spread evenly over the output range, 50
// to 253.  If COST_FACTOR is higher, cost values will have a plateau
// around obstacles and the planner will then treat (for example) the
// whole width of a narrow hallway as equally undesirable and thus
// will not plan paths down the center.
/*
 * 传入的costmap成本值在0到252之间。当COST_NEUTRAL为50时，COST_因子需要约为0.8，以确保输入值均匀分布在输出范围50到253之间。
 * 如果COST_因子更高，障碍物周围的成本值将有一个稳定，规划器将（例如）将狭窄走廊的整个宽度视为同样不需要的，因此不会规划沿着中心的路径。
*/

#define COST_NEUTRAL 50		// Set this to "open space" value
#define COST_FACTOR 0.8		// Used for translating costs in NavFn::setCostmap()

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
//为代价类型起别名
#ifndef COSTTYPE
#define COSTTYPE unsigned char	// Whatever is used...
#endif

// potential defs
#define POT_HIGH 1.0e10		// unassigned cell potential

//优先级缓冲区大小
// priority buffers
#define PRIORITYBUFSIZE 10000

namespace navfn {
/**
    Navigation function call.
    \param costmap Cost map array, of type COSTTYPE; origin is upper left
           NOTE: will be modified to have a border of obstacle costs
    \param nx Width of map in cells
    \param ny Height of map in cells
    \param goal X,Y position of goal cell
    \param start X,Y position of start cell

    Returns length of plan if found, and fills an array with x,y interpolated
    positions at about 1/2 cell resolution; else returns 0.

函数的功能：生成在栅格地图的导航路径
函数的输入：
          costmap: 代价地图数组指针，这个代价地图的起点是位于左上角
          nx：代价地图数组的宽度
          ny：代价地图数组的高度
          start：规划路径的起点（栅格地图中）
          goal：规划路径的目标点（栅格地图中）
          plan：规划的路径
          nplan：规划的路径迭代的次数
函数的返回：
          如果找到路径则返回路径的长度，否则返回0

*/

int create_nav_plan_astar(COSTTYPE *costmap, int nx, int ny, int* goal, int* start, float *plan, int nplan);

/**
   * @class NavFn
   * @brief Navigation function class. Holds buffers for costmap, navfn map. Maps are pixel-based. Origin is upper left, x is right, y is down.
*/
class NavFn
{
public:
  /**
       * @brief  Constructs the planner
       * @param nx The x size of the map 宽度
       * @param ny The y size of the map 高度
       */
  NavFn(int nx, int ny);	// size of map

  ~NavFn();

  /**
       * @brief  Sets or resets the size of the map
       * @param nx The x size of the map
       * @param ny The y size of the map
       */
  /**< sets or resets the size of the map */
  //设置或者重置地图大小
  void setNavArr(int nx, int ny);

  /**
       * @brief  Set up the cost array for the planner, usually from ROS
       * @param cmap The costmap
       * @param isROS Whether or not the costmap is coming in in ROS format
       * @param allow_unknown Whether or not the planner should be allowed to plan through unknown space
       */
  /**< sets up the cost map */
  //设置规划所用的代价地图
  void setCostmap(const COSTTYPE *cmap, bool isROS=true, bool allow_unknown = true);

  /**
       * @brief  Calculates a plan using the A* heuristic, returns true if one is found
       * @return True if a plan is found, false otherwise
       */
  /**< calculates a plan, returns true if found */
  //采用A*算法规划路径，如果规划成功则返回true，否则返回false
  bool calcNavFnAstar();

  /**
       * @brief Caclulates the full navigation function using Dijkstra
       */
  //采用Dijkstra算法规划路径，这个函数是最终的调用函数，会根据是否选择A*的参数进行算法切换，同样如果找到路径则返回true
  /**< calculates the full navigation function */
  bool calcNavFnDijkstra(bool atStart = false);

  /**
       * @brief  Accessor for the x-coordinates of a path
       * @return The x-coordinates of a path
       */
  //返回找到路径点的x部分
  /**< x-coordinates of path */
  float *getPathX();

  /**
       * @brief  Accessor for the y-coordinates of a path
       * @return The y-coordinates of a path
       */
  //返回找到路径点的y部分
  /**< y-coordinates of path */
  float *getPathY();

  /**
       * @brief  Accessor for the length of a path
       * @return The length of a path
       */
  //返回找到路径点的数量
  /**< length of path, 0 if not found */
  int   getPathLen();

  /**
       * @brief  Gets the cost of the path found the last time a navigation function was computed
       * @return The cost of the last path found
       */
  /**< Return cost of path found the last time A* was called */
  //返回找到路径的代价
  float getLastPathCost();


  /** goal and start positions */
  /**
       * @brief  Sets the goal position for the planner. Note: the navigation cost field computed gives the cost to get to a given point from the goal, not from the start.
       * @param goal the goal position
       */
  //设置目标点
  void setGoal(int *goal);

  /**
       * @brief  Sets the start position for the planner. Note: the navigation cost field computed gives the cost to get to a given point from the goal, not from the start.
       * @param start the start position
       */
  //设置起始点
  void setStart(int *start);


  /**
       * @brief  Initialize cell k with cost v for propagation
       * @param k the cell to initialize
       * @param v the cost to give to the cell
       */
  /**< initialize cell <k> with cost <v>, for propagation */
  //初始第k个栅格的代价为v
  void initCost(int k, float v);

  /** propagation */

  /**
       * @brief  Updates the cell at index n
       * @param n The index to update
       */
  /**< updates the cell at index <n> */
  //更新第n个栅格
  void updateCell(int n);

  /**
       * @brief  Updates the cell at index n using the A* heuristic
       * @param n The index to update
       */
  /**< updates the cell at index <n>, uses A* heuristic */
  //使用A*算法更新第n个栅格
  void updateCellAstar(int n);

  /**< resets all nav fn arrays for propagation */
  void setupNavFn(bool keepit = false);

  /**
       * @brief  Run propagation for <cycles> iterations, or until start is reached using breadth-first Dijkstra method
       * @param cycles The maximum number of iterations to run for
       * @param atStart Whether or not to stop when the start point is reached
       * @return true if the start point is reached
       */
  /**< returns true if start point found or full prop */
  //路径规划过程的传播函数
  bool propNavFnDijkstra(int cycles, bool atStart = false);


  /**
       * @brief  Run propagation for <cycles> iterations, or until start is reached using the best-first A* method with Euclidean distance heuristic
       * @param cycles The maximum number of iterations to run for
       * @return true if the start point is reached
       */
  /**< returns true if start point found */
  //路径规划过程的传播函数, 使用A*
  bool propNavFnAstar(int cycles);

  /**
       * @brief  Calculates the path for at mose <n> cycles
       * @param n The maximum number of cycles to run for
       * @return The length of the path found
       */
  /**< calculates path for at most <n> cycles, returns path length, 0 if none */
  //计算n次迭代后的路径
  int calcPath(int n, int *st = NULL);

  /**< calculates gradient at cell <n>, returns norm */
  //计算第n个栅格的梯度
  float gradCell(int n);

  /** display callback */
  /**< <n> is the number of cycles between updates  */
  //每n次输出路径迭代的结果
  void display(void fn(NavFn *nav), int n = 100);

  /** save costmap */
  /**< write out costmap and start/goal states as fname.pgm and fname.txt */
  //保存代价地图
  void savemap(const char *fname);


  /**< size of grid, in pixels */
  int nx;//The x size of the map 宽度
  int ny;//The y size of the map 高度
  int ns;//代价地图的栅格总数

  /** cell arrays */
  /**< cost array in 2D configuration space */
  COSTTYPE *costarr;//代价地图的数据

  /**< potential array, navigation function potential */
  float   *potarr;//???

  /**< pending cells during propagation */
  bool    *pending; //传播期间挂起的单元格

  /**< number of obstacle cells */
  int nobs;//障碍物栅格的数量

  /** block priority buffers */
  /**< storage buffers for priority blocks */
  int *pb1;
  int *pb2;
  int *pb3;

  /**< priority buffer block ptrs */
  int *curP;
  int *nextP;

  /**< end points of arrays */
  int *overP;

  int curPe;
  int nextPe;
  int overPe;

  /** block priority thresholds */
  /**< current threshold */
  float curT;

  /**< priority threshold increment */
  //优先级阈值增量
  float priInc;

  int goal[2];  //保存目标点的数组
  int start[2]; //保存起点的数组

  /** gradient and paths */
  /**< gradient arrays, size of potential array */
  float *gradx, *grady;//梯度数组

  /**< path points, as subpixel cell coordinates */
  float *pathx, *pathy;//找到路径的数组

  /**< number of path points */
  int npath; //找到路径的数量

  /**< size of pathx, pathy buffers */
  int npathbuf;//存储路径点的缓冲区大小

  /**< Holds the cost of the path found the last time A* was called */
  float last_path_cost_;//路径的代价

  ///**< step size for following gradient */
  //路径的步长
  float pathStep;

  /**< save second argument of display() above */
  //保存显示的第二个参数
  int displayInt;

  /**< display function itself */
  //函数指针
  void (*displayFn)(NavFn *nav);
};
}


#endif  // NAVFN
