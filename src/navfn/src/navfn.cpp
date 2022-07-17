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
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
//
// Path calc has sanity check that it succeeded
//


#include <navfn/navfn.h>
#include <ros/console.h>

namespace navfn {


// inserting onto the priority blocks
#define push_cur(n)  { if (n>=0 && n<ns && !pending[n] && \
  costarr[n]<COST_OBS && curPe<PRIORITYBUFSIZE) \
{ curP[curPe++]=n; pending[n]=true; }}

#define push_next(n) { if (n>=0 && n<ns && !pending[n] && \
  costarr[n]<COST_OBS && nextPe<PRIORITYBUFSIZE) \
{ nextP[nextPe++]=n; pending[n]=true; }}

#define push_over(n) { if (n>=0 && n<ns && !pending[n] && \
  costarr[n]<COST_OBS && overPe<PRIORITYBUFSIZE) \
{ overP[overPe++]=n; pending[n]=true; }}

//
// function to perform nav fn calculation
// keeps track of internal buffers, will be more efficient
//   if the size of the environment does not change
//
//这个函数是定义在类之外
int create_nav_plan_astar(COSTTYPE *costmap, int nx, int ny, int* goal, int* start, float *plan, int nplan) {

  //S1: 获取路径规划的类对象，这个类对象是定义成静态变量
  static NavFn *nav = NULL;//静态变量，第一次运行将定义和声明，第二次运行将直接访问

  if (nav == NULL)
    nav = new NavFn(nx,ny);//第一次会创建一个NavFn类对象

  //S2: 如果上一次的代价地图和当前的代价地图宽度和高度不一样，则重新创建路径规划的类对象
  if (nav->nx != nx || nav->ny != ny) // check for compatibility with previous call
  {
    delete nav;
    nav = new NavFn(nx,ny);
  }
  //S3: 设置路径规划的起点和目标点
  nav->setGoal(goal);
  nav->setStart(start);

  //S4: 设置路径规划所用的代价地图
  nav->costarr = costmap;

  //S5: 重置所有的指针变量
  nav->setupNavFn(true);

  // calculate the nav fn and path
  //S6: 设置优先级阈值增量为2 * 50
  nav->priInc = 2*COST_NEUTRAL;//COST_NEUTRAL = 50;

  //S7: 使用A*路径规划算法
  nav->propNavFnAstar(std::max(nx*ny/20,nx+ny));

  // path
  //S8: 获取规划路径的长度
  int len = nav->calcPath(nplan);

  //S9: 通过检查路径点的数量去确定是否获取到路径
  if (len > 0)	ROS_DEBUG("[NavFn] Path found, %d steps\n", len);		// found plan
  else ROS_DEBUG("[NavFn] No path found\n");

  //S10: 保存获取到的路径
  if (len > 0)
  {
    for (int i = 0; i<len; i++)
    {
      // 二维路径
      plan[i*2] = nav->pathx[i];
      plan[i*2+1] = nav->pathy[i];
    }
  }
  return len;
}


//
// create nav fn buffers
//
//类对象的构造函数
NavFn::NavFn(int xs, int ys) {

  //S1: 创建栅格数组指针 create cell arrays
  costarr = NULL; //代价地图的数据
  potarr = NULL;
  pending = NULL; //传播期间挂起的单元格
  gradx = grady = NULL; //梯度数组
  setNavArr(xs,ys); //根据栅格地图的宽度和高度释放上一次的指针变量的地址和生成这次指针变量的地址

  //S2: 创建3个优先缓冲区 priority buffers
  pb1 = new int[PRIORITYBUFSIZE];//PRIORITYBUFSIZE = 10000
  pb2 = new int[PRIORITYBUFSIZE];
  pb3 = new int[PRIORITYBUFSIZE];

  // for Dijkstra (breadth-first), set to COST_NEUTRAL // Dijkstra是广度优先
  // for A* (best-first), set to COST_NEUTRAL //A*是深度优先
  //S3: 设置优先级阈值增量为2 * 50
  priInc = 2*COST_NEUTRAL;

  //S4: 设置起点和目标点
  // goal and start
  goal[0] = goal[1] = 0;
  start[0] = start[1] = 0;

  // display function
  //S5: 设置显示函数指针
  displayFn = NULL; //输入迭代信息的函数指针
  displayInt = 0;//保存显示的第二个参数

  // path buffers
  //设置路径缓冲区大小及路径的数组指针
  npathbuf = npath = 0; //存储路径点的缓冲区大小
  pathx = pathy = NULL; //找到路径的数组
  pathStep = 0.5; //路径的步长
}


NavFn::~NavFn() {

  //析构函数释放已经分配内存的指针变量
  if(costarr)
    delete[] costarr;
  if(potarr)
    delete[] potarr;
  if(pending)
    delete[] pending;
  if(gradx)
    delete[] gradx;
  if(grady)
    delete[] grady;
  if(pathx)
    delete[] pathx;
  if(pathy)
    delete[] pathy;
  if(pb1)
    delete[] pb1;
  if(pb2)
    delete[] pb2;
  if(pb3)
    delete[] pb3;
}


//
// set goal, start positions for the nav fn
//
//设置目标点，栅格的坐标
void NavFn::setGoal(int *g) {

  goal[0] = g[0];
  goal[1] = g[1];
  ROS_DEBUG("[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
}

//设置起点，栅格的坐标
void NavFn::setStart(int *g) {
  start[0] = g[0];
  start[1] = g[1];
  ROS_DEBUG("[NavFn] Setting start to %d,%d\n", start[0], start[1]);
}

//
// Set/Reset map size
//

void NavFn::setNavArr(int xs, int ys) {

  //S1: 输出配置代价地图的宽度和高度
  ROS_DEBUG("[NavFn] Array is %d x %d\n", xs, ys);

  //S2: 初始化代价地图的宽度、高度和总的栅格数量
  nx = xs; //The x size of the map 宽度
  ny = ys; //The y size of the map 高度
  ns = nx*ny; //代价地图的栅格总数

  //S3: 释放上一次所用指针变量的地址
  if(costarr)//代价地图的数据
    delete[] costarr;
  if(potarr)
    delete[] potarr;
  if(pending)
    delete[] pending;//传播期间挂起的单元格

  if(gradx)//梯度数组
    delete[] gradx;
  if(grady)
    delete[] grady;

  //S4: 生成这次指针变量的地址
  costarr = new COSTTYPE[ns]; // cost array, 2d config space
  memset(costarr, 0, ns*sizeof(COSTTYPE));

  potarr = new float[ns];	// navigation potential array

  pending = new bool[ns];
  memset(pending, 0, ns*sizeof(bool));

  gradx = new float[ns];
  grady = new float[ns];
}


//
// set up cost array, usually from ROS
//
//设置代价地图
void NavFn::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown) {

  //S1: 获取类对象中代价地图的指针变量
  COSTTYPE *cm = costarr; //代价地图的指针变量
  //S2: 根据是否是ROS格式进行分别处理
  if (isROS)			// ROS-type cost array
  {
    //ROS格式的代价地图
    for (int i = 0; i< ny; i++) //ny为高度， 代价地图的起点是左上角
    {
      int k = i * nx; //nx是宽度, 第i行的栅格起点位置
      for (int j = 0; j < nx; j++, k++, cmap++, cm++)
      {
        // This transforms the incoming cost values:
        // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")
        // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated obstacle")
        // values in range 0 to 252 -> values from COST_NEUTRAL to COST_OBS_ROS.

        /*
         * 这里分三种情况：
                  情况1： 本身是禁止区域
                  情况2： 未知区域
                  情况3： 障碍物区域空闲区域
                  对于情况1和2，采用的是设置为禁止区域，对于情况3，采用的是求代价的公式，使代价为0到253
        */
        *cm = COST_OBS; //地图边界是禁止的区域 COST_OBS = 254

        int v = *cmap;  //通过解引用获取代价地图的数据

        if (v < COST_OBS_ROS) //COST_OBS_ROS = 253
        {
          v = COST_NEUTRAL + COST_FACTOR * v; // 计算代价 将代价归一化到0-253 cost = COST_NEUTRAL + COST_FACTOR * costmap_cost_value
          if (v >= COST_OBS) v = COST_OBS - 1;  //COST_OBS = 254
          *cm = v;
        }

        else if(v == COST_UNKNOWN_ROS && allow_unknown) //COST_UNKNOWN_ROS = 255;
        {
          v = COST_OBS - 1; //如果代价地图中是未知区域，都设置为禁止区域
          *cm = v;
        }

      }
    }
  }

  else				// not a ROS map, just a PGM
  {
    for (int i = 0; i < ny; i++)
    {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++)
      {
        *cm = COST_OBS;
        //这里是主要区别的地方
        if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) continue;	// don't do borders
        int v = *cmap;
        if (v < COST_OBS_ROS)
        {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS)
            v = COST_OBS - 1;
          *cm = v;
        }
        else if(v == COST_UNKNOWN_ROS)
        {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  }
}

bool NavFn::calcNavFnDijkstra(bool atStart) {

  setupNavFn(true);

  // calculate the nav fn and path
  propNavFnDijkstra(std::max(nx * ny/20,nx + ny),atStart);

  // path
  int len = calcPath(nx * ny / 2);

  if (len > 0)			// found plan
  {
    ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
    return true;
  }
  else
  {
    ROS_DEBUG("[NavFn] No path found\n");
    return false;
  }

}


//
// calculate navigation function, given a costmap, goal, and start
//

bool
NavFn::calcNavFnAstar()
{
  setupNavFn(true);

  // calculate the nav fn and path
  propNavFnAstar(std::max(nx*ny/20,nx+ny));

  // path
  int len = calcPath(nx*4);

  if (len > 0)			// found plan
  {
    ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
    return true;
  }
  else
  {
    ROS_DEBUG("[NavFn] No path found\n");
    return false;
  }
}


//
// returning values
//

float *NavFn::getPathX() { return pathx; }
float *NavFn::getPathY() { return pathy; }
int    NavFn::getPathLen() { return npath; }


// Set up navigation potential arrays for new propagation

void NavFn::setupNavFn(bool keepit) {

  // reset values in propagation arrays
  for (int i = 0; i < ns; i++) //ns是所有栅格的数量
  {
    potarr[i] = POT_HIGH;//POT_HIGH = 1.0e10 初值为很大的量
    if (!keepit) costarr[i] = COST_NEUTRAL; //COST_NEUTRAL = 50
    gradx[i] = grady[i] = 0.0; //所有的梯度都置为0
  }

  // outer bounds of cost array
  COSTTYPE *pc;
  pc = costarr;
  //设置代价地图的边界
  //最上面的边界
  for (int i = 0; i < nx; i++) *pc++ = COST_OBS;

  //最下面的边界
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++)  *pc++ = COST_OBS;

  //最左边的边界
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx)

  //最右边的边界
  *pc = COST_OBS;
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx)
    *pc = COST_OBS;

  // priority buffers
  curT = COST_OBS;
  curP = pb1;
  curPe = 0;
  nextP = pb2;
  nextPe = 0;
  overP = pb3;
  overPe = 0;
  memset(pending, 0, ns * sizeof(bool));

  // set goal
  int k = goal[0] + goal[1] * nx;
  initCost(k,0);

  // find # of obstacle cells
  pc = costarr;
  int ntot = 0;
  for (int i = 0; i < ns; i++, pc++)
  {
    if (*pc >= COST_OBS) //COST_OBS = 254
      ntot++;			// number of cells that are obstacles
  }
  nobs = ntot;
}


// initialize a goal-type cost for starting propagation

void NavFn::initCost(int k, float v) {

  potarr[k] = v;
  push_cur(k+1);
  push_cur(k-1);
  push_cur(k-nx);
  push_cur(k+nx);
}


//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void NavFn::updateCell(int n) {
  // get neighbors
  float u,d,l,r;
  l = potarr[n - 1];//当前栅格的左边栅格
  r = potarr[n + 1];//当前栅格的右边栅格
  u = potarr[n - nx];//当前栅格的上面栅格
  d = potarr[n + nx];//当前栅格的下面栅格
  //  ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
  //	 potarr[n], l, r, u, d);
  //  ROS_INFO("[Update] cost: %d\n", costarr[n]);

  // find lowest, and its lowest neighbor
  float ta, tc;
  if (l < r) tc = l; else tc = r; //找出左右两个方向最小的
  if (u < d) ta = u; else ta = d; //找出上下两个方向最小的

  // do planar wave update
  if (costarr[n] < COST_OBS)	// don't propagate into obstacles COST_OBS = 254
  {
    float hf = (float)costarr[n]; // traversability factor
    //左右和上下四个方向中找出最小值
    float dc = tc - ta;		// relative cost between ta,tc
    if (dc < 0) 		// ta is lowest
    {
      dc = -dc;
      ta = tc;
    }

    // calculate new potential
    float pot;
    if (dc >= hf)		// if too large, use ta-only update
      pot = ta + hf;
    else			// two-neighbor interpolation update
    {
      // use quadratic approximation
      // might speed this up through table lookup, but still have to
      //   do the divide
      float d = dc / hf;
      float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
      pot = ta + hf * v;
    }

    //      ROS_INFO("[Update] new pot: %d\n", costarr[n]);

    // now add affected neighbors to priority blocks
    if (pot < potarr[n])
    {
      float le = INVSQRT2 * (float)costarr[n - 1];
      float re = INVSQRT2 * (float)costarr[n + 1];
      float ue = INVSQRT2 * (float)costarr[n - nx];
      float de = INVSQRT2 * (float)costarr[n + nx];
      potarr[n] = pot;
      if (pot < curT)	// low-cost buffer block
      {
        if (l > pot + le) push_next(n - 1);
        if (r > pot + re) push_next(n + 1);
        if (u > pot + ue) push_next(n - nx);
        if (d > pot + de) push_next(n + nx);
      }
      else			// overflow block
      {
        if (l > pot + le) push_over(n - 1);
        if (r > pot + re) push_over(n + 1);
        if (u > pot + ue) push_over(n - nx);
        if (d > pot + de) push_over(n + nx);
      }
    }

  }

}


//
// Use A* method for setting priorities
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void
NavFn::updateCellAstar(int n)
{
  // get neighbors
  float u,d,l,r;
  l = potarr[n-1];
  r = potarr[n+1];
  u = potarr[n-nx];
  d = potarr[n+nx];
  //ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
  //	 potarr[n], l, r, u, d);
  // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

  // find lowest, and its lowest neighbor
  float ta, tc;
  if (l<r) tc=l; else tc=r;
  if (u<d) ta=u; else ta=d;

  // do planar wave update
  if (costarr[n] < COST_OBS)	// don't propagate into obstacles
  {
    float hf = (float)costarr[n]; // traversability factor
    float dc = tc-ta;		// relative cost between ta,tc
    if (dc < 0) 		// ta is lowest
    {
      dc = -dc;
      ta = tc;
    }

    // calculate new potential
    float pot;
    if (dc >= hf)		// if too large, use ta-only update
      pot = ta+hf;
    else			// two-neighbor interpolation update
    {
      // use quadratic approximation
      // might speed this up through table lookup, but still have to
      //   do the divide
      float d = dc/hf;
      float v = -0.2301*d*d + 0.5307*d + 0.7040;
      pot = ta + hf*v;
    }

    //ROS_INFO("[Update] new pot: %d\n", costarr[n]);

    // now add affected neighbors to priority blocks
    if (pot < potarr[n])
    {
      float le = INVSQRT2*(float)costarr[n-1];
      float re = INVSQRT2*(float)costarr[n+1];
      float ue = INVSQRT2*(float)costarr[n-nx];
      float de = INVSQRT2*(float)costarr[n+nx];

      // calculate distance
      int x = n%nx;
      int y = n/nx;
      float dist = hypot(x-start[0], y-start[1])*(float)COST_NEUTRAL;

      potarr[n] = pot;
      pot += dist;
      if (pot < curT)	// low-cost buffer block
      {
        if (l > pot+le) push_next(n-1);
        if (r > pot+re) push_next(n+1);
        if (u > pot+ue) push_next(n-nx);
        if (d > pot+de) push_next(n+nx);
      }
      else
      {
        if (l > pot+le) push_over(n-1);
        if (r > pot+re) push_over(n+1);
        if (u > pot+ue) push_over(n-nx);
        if (d > pot+de) push_over(n+nx);
      }
    }

  }

}



//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool NavFn::propNavFnDijkstra(int cycles, bool atStart) {

  int nwv = 0;			// max priority block size //最大优先级缓冲区的大小
  int nc = 0;			// number of cells put into priority blocks//已经放进优先级缓冲区的栅格数量
  int cycle = 0;		// which cycle we're on//当前路径规划的次数

  // set up start cell
  //S1: 获取起点对应的代价地图的索引
  int startCell = start[1] * nx + start[0]; //路径起点的栅格索引, start[1]是y-高度，start[0]是x-宽度, nx是代价地图的宽度
  //S2: 开始规划路径
  for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
  {
    //S21: 检查优先级缓冲区是否为空，如果为空就退出路径规划，因为当前可以选择的路径点
    if (curPe == 0 && nextPe == 0) // priority blocks empty
      break;

    // stats
    nc += curPe;

    if (curPe > nwv)
      nwv = curPe;

    // reset pending flags on current priority buffer
    int *pb = curP;
    int i = curPe;
    while (i-- > 0)
      pending[*(pb++)] = false;

    // process current priority buffer
    pb = curP;
    i = curPe;
    while (i-- > 0)
      updateCell(*pb++);

    if (displayInt > 0 &&  (cycle % displayInt) == 0)
      displayFn(this);

    // swap priority blocks curP <=> nextP
    curPe = nextPe;
    nextPe = 0;
    pb = curP;		// swap buffers
    curP = nextP;
    nextP = pb;

    // see if we're done with this priority level
    if (curPe == 0)
    {
      curT += priInc;	// increment priority threshold
      curPe = overPe;	// set current to overflow block
      overPe = 0;
      pb = curP;		// swap buffers
      curP = overP;
      overP = pb;
    }

    // check if we've hit the Start cell
    if (atStart)
      if (potarr[startCell] < POT_HIGH)
        break;
  }

  ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
            cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);

  if (cycle < cycles) return true; // finished up here
  else return false;
}


//
// main propagation function
// A* method, best-first
// uses Euclidean distance heuristic
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool
NavFn::propNavFnAstar(int cycles)
{
  int nwv = 0;			// max priority block size
  int nc = 0;			// number of cells put into priority blocks
  int cycle = 0;		// which cycle we're on

  // set initial threshold, based on distance
  float dist = hypot(goal[0]-start[0], goal[1]-start[1])*(float)COST_NEUTRAL;
  curT = dist + curT;

  // set up start cell
  int startCell = start[1]*nx + start[0];

  // do main cycle
  for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
  {
    //
    if (curPe == 0 && nextPe == 0) // priority blocks empty
      break;

    // stats
    nc += curPe;
    if (curPe > nwv)
      nwv = curPe;

    // reset pending flags on current priority buffer
    int *pb = curP;
    int i = curPe;
    while (i-- > 0)
      pending[*(pb++)] = false;

    // process current priority buffer
    pb = curP;
    i = curPe;
    while (i-- > 0)
      updateCellAstar(*pb++);

    if (displayInt > 0 &&  (cycle % displayInt) == 0)
      displayFn(this);

    // swap priority blocks curP <=> nextP
    curPe = nextPe;
    nextPe = 0;
    pb = curP;		// swap buffers
    curP = nextP;
    nextP = pb;

    // see if we're done with this priority level
    if (curPe == 0)
    {
      curT += priInc;	// increment priority threshold
      curPe = overPe;	// set current to overflow block
      overPe = 0;
      pb = curP;		// swap buffers
      curP = overP;
      overP = pb;
    }

    // check if we've hit the Start cell
    if (potarr[startCell] < POT_HIGH)
      break;

  }

  last_path_cost_ = potarr[startCell];

  ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
            cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);


  if (potarr[startCell] < POT_HIGH) return true; // finished up here
  else return false;
}


float NavFn::getLastPathCost()
{
  return last_path_cost_;
}


//
// Path construction
// Find gradient at array points, interpolate path
// Use step size of pathStep, usually 0.5 pixel
//
// Some sanity checks:
//  1. Stuck at same index position
//  2. Doesn't get near goal
//  3. Surrounded by high potentials
//

int NavFn::calcPath(int n, int *st) {
  // test write
  //savemap("test");

  // check path arrays
  //S1: 检查路径缓冲区的大小，如果小于指定的则释放一开始初始化的指针，重新为新指针变量分配内存
  if (npathbuf < n) //存储路径点的缓冲区大小
  {
    if (pathx) delete [] pathx;
    if (pathy) delete [] pathy;
    pathx = new float[n];
    pathy = new float[n];
    npathbuf = n;
  }

  // set up start position at cell
  // st is always upper left corner for 4-point bilinear interpolation 对于4点双线性插值，st始终为左上角
  // 如果没有指定st，则st采用类中的成员
  //S2: 获取路径规划的起始点
  if (st == NULL) st = start;

  int stc = st[1] * nx + st[0];//stc是地图数据的索引, nx是地图的宽度

  // set up offset
  //S3: 设置偏移量
  float dx=0; // 找路径的x方向的偏移量 x是宽度
  float dy=0; // 找路径的y方向的偏移量 y是高度
  npath = 0;

  // go for <n> cycles at most
  //S4: 最大迭代n次去寻找路径
  for (int i = 0; i< n; i++)
  {
    // check if near goal
    //S41: 检查是否靠近目标点, stc是起点的位置 //(nx * ny - 1): 整个栅格的最大值, 0: 0：整个栅格的最小值
    int nearest_point = std::max(0, std::min(nx * ny - 1, stc + (int)round(dx) + (int)(nx * round(dy)))); //保证当前的索引在有效的范围内

    if (potarr[nearest_point] < COST_NEUTRAL) //位于目标点附近，如何判断的？
    {
      pathx[npath] = (float)goal[0];
      pathy[npath] = (float)goal[1];
      return ++npath;	// done!
    }
    //S42: 检查起点是否超出地图的边界
    if (stc < nx || stc > ns-nx) // would be out of bounds //检查上边界和下边界，那左边界和右边界？
    {
      ROS_DEBUG("[PathCalc] Out of bounds");
      return 0;
    }

    //S43: 将路径点添加到路径数组中
    // add to path
    pathx[npath] = stc % nx + dx;
    pathy[npath] = stc / nx + dy;
    npath++;
    //S44: 检查已找到的路径点是否存在振荡即来回运动
    bool oscillation_detected = false;
    if( npath > 2 && pathx[npath - 1] == pathx[npath - 3] && pathy[npath - 1] == pathy[npath - 3] )
    {
      ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
      oscillation_detected = true;
    }

    int stcnx = stc + nx;//stc栅格对应的下栅格
    int stcpx = stc - nx;//stc栅格对应的上栅格
    //S45: 检查路径点附近的八个点的状态
    // check for potentials at eight positions near cell
    if (potarr[stc] >= POT_HIGH ||
        potarr[stc+1] >= POT_HIGH ||
        potarr[stc-1] >= POT_HIGH ||
        potarr[stcnx] >= POT_HIGH ||
        potarr[stcnx+1] >= POT_HIGH ||
        potarr[stcnx-1] >= POT_HIGH ||
        potarr[stcpx] >= POT_HIGH ||
        potarr[stcpx+1] >= POT_HIGH ||
        potarr[stcpx-1] >= POT_HIGH ||
        oscillation_detected)
    {
      ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);
      //从八个邻居中找到最小的
      // check eight neighbors to find the lowest
      int minc = stc;
      int minp = potarr[stc];
      int st = stcpx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
      st = stc-1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
      st = stc+1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
      st = stcnx-1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
      stc = minc;
      dx = 0;
      dy = 0;

      ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
                potarr[stc], pathx[npath-1], pathy[npath-1]);

      if (potarr[stc] >= POT_HIGH) //如果最小的都是禁止区域，则找不到路径，直接返回
      {
        ROS_DEBUG("[PathCalc] No path found, high potential");
        //savemap("navfn_highpot");
        return 0;
      }
    }

    // have a good gradient here
    else
    {

      // get grad at four positions near cell
      gradCell(stc);//stc是当前对应的栅格
      gradCell(stc+1);//stc右边的栅格
      gradCell(stcnx);//stcnx是stc栅格对应的下栅格
      gradCell(stcnx+1);//stc栅格的下栅格的右栅格


      // get interpolated gradient
      float x1 = (1.0 - dx) * gradx[stc] + dx * gradx[stc + 1];
      float x2 = (1.0 - dx) * gradx[stcnx] + dx * gradx[stcnx + 1];
      float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
      float y1 = (1.0 - dx) * grady[stc] + dx * grady[stc + 1];
      float y2 = (1.0 - dx) * grady[stcnx] + dx * grady[stcnx + 1];
      float y = (1.0 - dy) * y1 + dy * y2; // interpolated y

      // show gradients
      ROS_DEBUG("[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
                gradx[stc], grady[stc], gradx[stc+1], grady[stc+1],
          gradx[stcnx], grady[stcnx], gradx[stcnx+1], grady[stcnx+1],
          x, y);

      // check for zero gradient, failed
      if (x == 0.0 && y == 0.0)
      {
        ROS_DEBUG("[PathCalc] Zero gradient");
        return 0;
      }

      // move in the right direction
      float ss = pathStep / hypot(x, y);
      dx += x * ss;
      dy += y * ss;

      // check for overflow
      if (dx > 1.0) { stc++; dx -= 1.0; }
      if (dx < -1.0) { stc--; dx += 1.0; }
      if (dy > 1.0) { stc += nx; dy -= 1.0; }
      if (dy < -1.0) { stc -= nx; dy += 1.0; }

    }

    //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
    //	     potarr[stc], x, y, pathx[npath-1], pathy[npath-1]);
  }

  //  return npath;			// out of cycles, return failure
  ROS_DEBUG("[PathCalc] No path found, path too long");
  //savemap("navfn_pathlong");
  return 0;			// out of cycles, return failure
}


//
// gradient calculations
//

// calculate gradient at a cell
// positive value are to the right and down
// 计算第n个栅格的梯度值
float NavFn::gradCell(int n) {

  if (gradx[n] + grady[n] > 0.0)	// check this cell
    return 1.0;

  if (n < nx || n > ns-nx)	// would be out of bounds //最上层和最下层
    return 0.0;

  float cv = potarr[n];

  float dx = 0.0;
  float dy = 0.0;

  // check for in an obstacle
  if (cv >= POT_HIGH)
  {
    //因为第n个栅格是障碍物，所以前面一个栅格和后面一个栅格，左面一个栅格和右面一个栅格都要设置成禁止区域
    if (potarr[n - 1] < POT_HIGH) dx = -COST_OBS;
    else if (potarr[n + 1] < POT_HIGH) dx = COST_OBS;

    if (potarr[n - nx] < POT_HIGH) dy = -COST_OBS;
    else if (potarr[n + nx] < POT_HIGH) dy = COST_OBS;
  }

  else				// not in an obstacle
  {
    // dx calc, average to sides//前一个减去后一个
    if (potarr[n - 1] < POT_HIGH) dx += potarr[n - 1] - cv;
    if (potarr[n + 1] < POT_HIGH) dx += cv - potarr[n + 1];

    // dy calc, average to sides//上一个减去下一个
    if (potarr[n - nx] < POT_HIGH) dy += potarr[n - nx] - cv;
    if (potarr[n + nx] < POT_HIGH) dy += cv - potarr[n + nx];
  }

  // normalize
  float norm = hypot(dx, dy); //sqrt(X*X + Y*Y)
  //将梯度归一化到-1到1
  if (norm > 0)
  {
    norm = 1.0 / norm;
    gradx[n] = norm * dx;
    grady[n] = norm * dy;
  }
  return norm;
}


//
// display function setup
// <n> is the number of cycles to wait before displaying,
//     use 0 to turn it off

void NavFn::display(void fn(NavFn *nav), int n) {

  displayFn = fn;
  displayInt = n;
}


//
// debug writes
// saves costmap and start/goal
//
//保存代价地图到文件fname.pgm，起点和终点到文件fname.txt
void NavFn::savemap(const char *fname) {

  char fn[4096]; //创建一个字符数组去保存文件的名字

  ROS_DEBUG("[NavFn] Saving costmap and start/goal points");
  // write start and goal points

  sprintf(fn, "%s.txt", fname); //将fname.txt保存到fn中

  FILE *fp = fopen(fn, "w"); //以写模式打开一个文件

  if (!fp)
  {
    ROS_WARN("Can't open file %s", fn);
    return;
  }

  fprintf(fp, "Goal: %d %d\nStart: %d %d\n", goal[0], goal[1], start[0], start[1]);//
  fclose(fp);

  // write cost array
  if (!costarr) return;

  sprintf(fn, "%s.pgm", fname);

  fp = fopen(fn, "wb");
  if (!fp)
  {
    ROS_WARN("Can't open file %s", fn);
    return;
  }
  fprintf(fp,"P5\n%d\n%d\n%d\n", nx, ny, 0xff);
  fwrite(costarr, 1, nx * ny, fp);
  fclose(fp);
}
}
