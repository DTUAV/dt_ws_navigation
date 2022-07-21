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

#ifndef SIMPLE_TRAJECTORY_GENERATOR_H_
#define SIMPLE_TRAJECTORY_GENERATOR_H_

#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/local_planner_limits.h>
#include <Eigen/Core>
//轨迹生成器
namespace base_local_planner {

/**
 * generates trajectories based on equi-distant discretisation of the degrees of freedom.
 * This is supposed to be a simple and robust implementation of the TrajectorySampleGenerator
 * interface, more efficient implementations are thinkable.
 *
 * This can be used for both dwa and trajectory rollout approaches.
 * As an example, assuming these values:
 * sim_time = 1s, sim_period=200ms, dt = 200ms,
 * vsamples_x=5,
 * acc_limit_x = 1m/s^2, vel_x=0 (robot at rest, values just for easy calculations)
 * dwa_planner will sample max-x-velocities from 0m/s to 0.2m/s.
 * trajectory rollout approach will sample max-x-velocities 0m/s up to 1m/s
 * trajectory rollout approach does so respecting the acceleration limit, so it gradually increases velocity
 */
class SimpleTrajectoryGenerator: public base_local_planner::TrajectorySampleGenerator {
public:

  SimpleTrajectoryGenerator() {
    limits_ = NULL;
  }

  ~SimpleTrajectoryGenerator() {}

  /**
   * @param pos current robot position
   * @param vel current robot velocity
   * @param limits Current velocity limits
   * @param vsamples: in how many samples to divide the given dimension
   * @param use_acceleration_limits: if true use physical model, else idealized robot model
   * @param additional_samples (deprecated): Additional velocity samples to generate individual trajectories from.
   * @param discretize_by_time if true, the trajectory is split according in chunks of the same duration, else of same length
   */
  void initialise(
      const Eigen::Vector3f& pos, /*当前机器人的位姿*/
      const Eigen::Vector3f& vel, /*当前机器人的速度*/
      const Eigen::Vector3f& goal,/*机器人目标位姿*/
      base_local_planner::LocalPlannerLimits* limits,/*机器人的约束参数*/
      const Eigen::Vector3f& vsamples,/*划分给定维度的样本数，也就是x轴多少个点，y轴多少个点*/
      std::vector<Eigen::Vector3f> additional_samples,
      bool discretize_by_time = false);

  /**
   * @param pos current robot position
   * @param vel current robot velocity
   * @param limits Current velocity limits
   * @param vsamples: in how many samples to divide the given dimension
   * @param use_acceleration_limits: if true use physical model, else idealized robot model
   * @param discretize_by_time if true, the trajectory is split according in chunks of the same duration, else of same length
   */
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      const Eigen::Vector3f& goal,
      base_local_planner::LocalPlannerLimits* limits,
      const Eigen::Vector3f& vsamples,
      bool discretize_by_time = false);

  /**
   * This function is to be called only when parameters change
   *
   * @param sim_granularity granularity of collision detection
   * @param angular_sim_granularity angular granularity of collision detection
   * @param use_dwa whether to use DWA or trajectory rollout
   * @param sim_period distance between points in one trajectory
   */
  //设置主要的参数，模拟时间、模拟平移的间隔、模拟角度的间隔、是否使用dwa算法及dwa算法模拟周期
  void setParameters(double sim_time,
      double sim_granularity,
      double angular_sim_granularity,
      bool use_dwa = false,
      double sim_period = 0.0);

  /**
   * Whether this generator can create more trajectories
   */
  bool hasMoreTrajectories();

  /**
   * Whether this generator can create more trajectories
   */
  bool nextTrajectory(Trajectory &traj);

//根据当前位姿、速度、增量时间计算下一个位姿
  static Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel, double dt);

//根据当前速度、目标速度、加速度约束和增量时间，计算下一个速度
  static Eigen::Vector3f computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
      const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt);
//生成轨迹
  bool generateTrajectory(
        Eigen::Vector3f pos,
        Eigen::Vector3f vel,
        Eigen::Vector3f sample_target_vel,
        base_local_planner::Trajectory& traj);

protected:

  unsigned int next_sample_index_;//下一个采样点的索引
  // to store sample params of each sample between init and generation
  std::vector<Eigen::Vector3f> sample_params_;
  base_local_planner::LocalPlannerLimits* limits_;//局部规划器的约束参数
  Eigen::Vector3f pos_;//起始位置
  Eigen::Vector3f vel_;//起始速度

  // whether velocity of trajectory changes over time or not
  bool continued_acceleration_;//是否连续加速
  bool discretize_by_time_;//是否随时间离散化

  double sim_time_;//模拟的时间
  double sim_granularity_;//模拟的间隔
  double angular_sim_granularity_;//角速度模拟的间隔

  bool use_dwa_;//是否使用dwa算法
  double sim_period_; // only for dwa//dwa算法的模拟间隔
};

} /* namespace base_local_planner */
#endif /* SIMPLE_TRAJECTORY_GENERATOR_H_ */
