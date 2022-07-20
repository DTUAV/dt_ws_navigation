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
#ifndef _EXPANDER_H
#define _EXPANDER_H
#include <global_planner/potential_calculator.h>
#include <global_planner/planner_core.h>

namespace global_planner {
//这是规划器的基类
class Expander {
public:
  Expander(PotentialCalculator* p_calc, int nx, int ny) :
    unknown_(true), lethal_cost_(253), neutral_cost_(50), factor_(3.0), p_calc_(p_calc) {
    setSize(nx, ny);
  }
//由于这里是不仅考虑最短路径，而且考虑环境的避障，所以，规划器做出的决策要综合两种情况，也就是需要代价和势场。采用势场去描述安全性
  virtual bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                   int cycles, float* potential) = 0;

  /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
  //设置代价地图的宽度、高度和栅格总数
  virtual void setSize(int nx, int ny) {
    nx_ = nx;
    ny_ = ny;
    ns_ = nx * ny;
  } /**< sets or resets the size of the map */
  //设置致命代价值
  void setLethalCost(unsigned char lethal_cost) {
    lethal_cost_ = lethal_cost;
  }
  //设置中性代价值
  void setNeutralCost(unsigned char neutral_cost) {
    neutral_cost_ = neutral_cost;
  }
  //设置代价计算的系数
  void setFactor(float factor) {
    factor_ = factor;
  }
  //设置是否应允许规划师通过未知空间进行规划
  void setHasUnknown(bool unknown) {
    unknown_ = unknown;
  }

  void clearEndpoint(unsigned char* costs, float* potential, int gx, int gy, int s){

    int startCell = toIndex(gx, gy);//获取当前gx和gy对应的索引

    for(int i = -s; i <= s; i++) {
      for(int j = -s; j <= s; j++) {
        int n = startCell + i + nx_ * j;
        if(potential[n] < POT_HIGH) //当前n是未分配
          continue;
        float c = costs[n] + neutral_cost_;//计算代价
        float pot = p_calc_->calculatePotential(potential, c, n);
        potential[n] = pot;
      }
    }
  }

protected:
  //根据指定宽度和高度获取索引
  inline int toIndex(int x, int y) {
    return x + nx_ * y;
  }

  /**< size of grid, in pixels */
  int nx_; //代价地图的宽度x
  int ny_; //代价地图的高度y
  int ns_; //代价地图中栅格总数

  bool unknown_; //是否应允许规划师通过未知空间进行规划
  unsigned char lethal_cost_;//致命的代价，规划的路径要尽可以远离
  unsigned neutral_cost_;//中性代价，规划的路径要尽可能靠近
  int cells_visited_;   //已经访问过的栅格数
  float factor_;        //代价转换的系数
  PotentialCalculator* p_calc_; //主要计算规划点附近的代价，以便选择最小的方向前进

};

} //end namespace global_planner
#endif
