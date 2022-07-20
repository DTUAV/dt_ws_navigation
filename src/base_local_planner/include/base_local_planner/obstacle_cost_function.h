/*********************************************************************
 *
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
 * Author: TKruse
 *********************************************************************/

#ifndef OBSTACLE_COST_FUNCTION_H_
#define OBSTACLE_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>
//障碍物的代价函数 //如果机器人足迹在轨迹的任何点上处于障碍物中，则使用costmap 2d分配负成本
namespace base_local_planner {

/**
 * class ObstacleCostFunction
 * @brief Uses costmap 2d to assign negative costs if robot footprint
 * is in obstacle on any point of the trajectory.
 */
class ObstacleCostFunction : public TrajectoryCostFunction {

public:
  ObstacleCostFunction(costmap_2d::Costmap2D* costmap);
  ~ObstacleCostFunction();

  bool prepare();
  double scoreTrajectory(Trajectory &traj);

  void setSumScores(bool score_sums){ sum_scores_=score_sums; }

  void setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed);
  void setFootprint(std::vector<geometry_msgs::Point> footprint_spec);

  // helper functions, made static for easy unit testing
  static double getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor);
  static double footprintCost(
      const double& x,
      const double& y,
      const double& th,
      double scale,
      std::vector<geometry_msgs::Point> footprint_spec,
      costmap_2d::Costmap2D* costmap,
      base_local_planner::WorldModel* world_model);

private:
  costmap_2d::Costmap2D* costmap_;//代价地图
  std::vector<geometry_msgs::Point> footprint_spec_;//底盘位置数据
  base_local_planner::WorldModel* world_model_;//环境模型，这里主要是costmap_model
 //如果我们超过某个速度阈值，我们将缩放机器人的足迹，使其减速或远离墙壁
  //scaling_speed是缩放机器人底盘的速度阈值
  //max_scaling_factor是缩放的参数，如果速度大了就放大机器人的汽车底盘，给制动留更多的缓冲时间和安全裕度
  double max_trans_vel_;//最大的平移速度
  bool sum_scores_;//是否计算得分的总和
  //footprint scaling with velocity;
  double max_scaling_factor_, scaling_speed_;
};

} /* namespace base_local_planner */
#endif /* OBSTACLE_COST_FUNCTION_H_ */
