/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/dijkstra.h>
#include <algorithm>
namespace global_planner {

DijkstraExpansion::DijkstraExpansion(PotentialCalculator* p_calc, int nx, int ny) :
  Expander(p_calc, nx, ny), pending_(NULL), precise_(false) {
  // priority buffers//
  buffer1_ = new int[PRIORITYBUFSIZE];//每个缓冲区可容纳10000个栅格点，这个要根据实际地图的大小修改
  buffer2_ = new int[PRIORITYBUFSIZE];
  buffer3_ = new int[PRIORITYBUFSIZE];

  priorityIncrement_ = 2 * neutral_cost_;//优先级的增量为100
}

DijkstraExpansion::~DijkstraExpansion() {
  delete[] buffer1_;//析构函数，释放已分配的内存
  delete[] buffer2_;
  delete[] buffer3_;
  if (pending_)
    delete[] pending_;
}

//
// Set/Reset map size
//
void DijkstraExpansion::setSize(int xs, int ys) {
  Expander::setSize(xs, ys);
  if (pending_)
    delete[] pending_;

  pending_ = new bool[ns_];
  memset(pending_, 0, ns_ * sizeof(bool));//表示所有的栅格都没有被访问过
}

//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool DijkstraExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                            int cycles, float* potential) {
  cells_visited_ = 0;//当前没有栅格被访问过
  // priority buffers
  threshold_ = lethal_cost_;//阈值为50
  currentBuffer_ = buffer1_;//当前缓冲区的指针指向当前缓冲区
  currentEnd_ = 0;//当前缓冲区的末端为0，表示没有数据
  nextBuffer_ = buffer2_;//下一个缓冲区的指针指向下一个缓冲区
  nextEnd_ = 0;//下一个缓冲区的末端为0，表示没有数据
  overBuffer_ = buffer3_;//移出缓冲区的指针指向移出缓冲区
  overEnd_ = 0;//移出缓冲区的末端为0，表示没有数据
  memset(pending_, 0, ns_ * sizeof(bool));//表示是否访问的标志缓冲区中数据全设置为false
  std::fill(potential, potential + ns_, POT_HIGH);//:fill(iterator start, iterator end, value);//将每个栅格的势场都设置为无穷大

  // set goal
  int k = toIndex(start_x, start_y);//获得起点坐标的索引

  if(precise_) {

    double dx = start_x - (int)start_x, dy = start_y - (int)start_y;

    dx = floorf(dx * 100 + 0.5) / 100;
    dy = floorf(dy * 100 + 0.5) / 100;
    //为什么是设置右下角，因为代价地图的起点是左上角，也就是n是左上角的索引，只有右下角有值
    potential[k] = neutral_cost_ * 2 * dx * dy;// 50 * 2 * dx * dy

    potential[k + 1] = neutral_cost_ * 2 * (1 - dx) * dy;

    potential[k + nx_] = neutral_cost_ * 2 * dx * (1 - dy);

    potential[k + nx_ + 1] = neutral_cost_ * 2 * (1 - dx) * (1 - dy);//*/

    push_cur(k + 2);
    push_cur(k - 1);
    push_cur(k + nx_ - 1);
    push_cur(k + nx_ + 2);

    push_cur(k - nx_);
    push_cur(k - nx_ + 1);
    push_cur(k + nx_ * 2);
    push_cur(k + nx_ * 2 + 1);
  }
  else{
    potential[k] = 0;
    push_cur(k + 1);//右
    push_cur(k - 1);//左
    push_cur(k - nx_);//上
    push_cur(k + nx_);//下
  }
  //上面执行之后currentEnd_ != 0

  int nwv = 0;            // max priority block size
  int nc = 0;            // number of cells put into priority blocks
  int cycle = 0;        // which cycle we're on

  // set up start cell
  int startCell = toIndex(end_x, end_y);

  for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
  {
    //
    if (currentEnd_ == 0 && nextEnd_ == 0) // priority blocks empty
      return false;

    // stats
    nc += currentEnd_;
    if (currentEnd_ > nwv)
      nwv = currentEnd_;

    //更新标志数组中标志，当前缓冲区的数据为没有访问过
    // reset pending_ flags on current priority buffer
    int *pb = currentBuffer_;
    int i = currentEnd_;
    while (i-- > 0)
      pending_[*(pb++)] = false;

    //更新当前缓冲区栅格的代价和势场
    // process current priority buffer
    pb = currentBuffer_;
    i = currentEnd_;
    while (i-- > 0)
      updateCell(costs, potential, *pb++);

    //将下一个的缓冲区和当前缓冲区交换
    // swap priority blocks currentBuffer_ <=> nextBuffer_
    currentEnd_ = nextEnd_;
    nextEnd_ = 0;

    pb = currentBuffer_;        // swap buffers
    currentBuffer_ = nextBuffer_;

    nextBuffer_ = pb;

    //由于设置的阈值过小，导致路径点被过滤掉，所以这里是将过滤的阈值提高并且将当前缓冲区与移出缓冲区交换，从之前移出缓冲区中拿数据
    // see if we're done with this priority level
    if (currentEnd_ == 0) {
      threshold_ += priorityIncrement_;    // increment priority threshold
      currentEnd_ = overEnd_;    // set current to overflow block
      overEnd_ = 0;
      pb = currentBuffer_;        // swap buffers
      currentBuffer_ = overBuffer_;
      overBuffer_ = pb;
    }

    // check if we've hit the Start cell
    if (potential[startCell] < POT_HIGH)//当到达目标点时，目标点的势场会被更新，此时一旦检测到目标点的势场不是无穷大，就说明已经找到可行的路径
      break;
  }
  //ROS_INFO("CYCLES %d/%d ", cycle, cycles);
  if (cycle < cycles)//如果是找到目标路径出来的，cycle一定是小于cycles的
    return true; // finished up here
  else
    return false;
}

//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void DijkstraExpansion::updateCell(unsigned char* costs, float* potential, int n) {

  cells_visited_++;//更新一次，访问的栅格数加1
  // do planar wave update
  float c = getCost(costs, n);//获取当前的代价，从代价地图中获取

  if (c >= lethal_cost_)    // don't propagate into obstacles //不更新障碍物的势场，障碍物的势场为无穷大
    return;

  float pot = p_calc_->calculatePotential(potential, c, n);//获取上下左右四个方向的势场最小值并和当前的栅格值的势场相加，因为后面要往最小势场方向走

  // now add affected neighbors to priority blocks
  if (pot < potential[n]) { //pot是四个邻居中最小的一个，判断最小的一个是否小于当前的代价
    float le = INVSQRT2 * (float)getCost(costs, n - 1);
    float re = INVSQRT2 * (float)getCost(costs, n + 1);
    float ue = INVSQRT2 * (float)getCost(costs, n - nx_);
    float de = INVSQRT2 * (float)getCost(costs, n + nx_);
    potential[n] = pot;
    //ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]);
    if (pot < threshold_)    // low-cost buffer block //threshold_是一个代价阈值，如果阈值过大，则说明这条路径的危险度高，不选择这条路径
    {
      if (potential[n - 1] > pot + le) //如果的势场比通过当前栅格后的还大，说明通过当前栅格的路径是优的
        push_next(n - 1);
      if (potential[n + 1] > pot + re)
        push_next(n + 1);
      if (potential[n - nx_] > pot + ue)
        push_next(n - nx_);
      if (potential[n + nx_] > pot + de)
        push_next(n + nx_);
    }
    else            // overflow block
    {
      if (potential[n - 1] > pot + le)
        push_over(n - 1);
      if (potential[n + 1] > pot + re)
        push_over(n + 1);
      if (potential[n - nx_] > pot + ue)
        push_over(n - nx_);
      if (potential[n + nx_] > pot + de)
        push_over(n + nx_);
    }
  }
}

} //end namespace global_planner
