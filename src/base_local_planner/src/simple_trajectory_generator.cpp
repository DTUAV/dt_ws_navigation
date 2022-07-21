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

#include <base_local_planner/simple_trajectory_generator.h>

#include <cmath>

#include <base_local_planner/velocity_iterator.h>

namespace base_local_planner {

void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    std::vector<Eigen::Vector3f> additional_samples,
    bool discretize_by_time) {
  initialise(pos, vel, goal, limits, vsamples, discretize_by_time);
  // add static samples if any
  sample_params_.insert(sample_params_.end(), additional_samples.begin(), additional_samples.end());
}

//调用这个初始化函数就能获得所有可能轨迹
void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    bool discretize_by_time) {
  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  double max_vel_th = limits->max_vel_theta;//最大的角速度
  double min_vel_th = -1.0 * max_vel_th;//这里的最小角速度不是配置的，而是最大速度的负值!!!!!!!!!!!!!!!!
  discretize_by_time_ = discretize_by_time;//是否离散时间
  Eigen::Vector3f acc_lim = limits->getAccLimits();//获取加速度约束
  pos_ = pos;//当前的位姿（x,y,theta）
  vel_ = vel;//当前的速度
  limits_ = limits;//当前配置的局部路径规划约束参数
  next_sample_index_ = 0;//下一个采用的索引
  sample_params_.clear();//

  double min_vel_x = limits->min_vel_x;//x轴方向最小速度
  double max_vel_x = limits->max_vel_x;//x轴方向最大速度
  double min_vel_y = limits->min_vel_y;//y轴方向最小速度
  double max_vel_y = limits->max_vel_y;//y轴方向最大速度

  // if sampling number is zero in any dimension, we don't generate samples generically
  //如果配置三个维度的采样点小于等于0，则不进行轨迹采样
  if (vsamples[0] * vsamples[1] * vsamples[2] > 0) {
    //compute the feasible velocity space based on the rate at which we run
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();

    if ( ! use_dwa_) { //这里是不使用dwa算法
      // there is no point in overshooting the goal, and it also may break the
      // robot behavior, so we limit the velocities to those that do not overshoot in sim_time
      double dist = hypot(goal[0] - pos[0], goal[1] - pos[1]);//获取目标位置点和当前位置的距离

      max_vel_x = std::max(std::min(max_vel_x, dist / sim_time_), min_vel_x);//这里当距离除以模拟时间的值在速度的约束范围内，以这个值作为机器人运动的最大速度。因为距离最大时对应的速度也应最大，后面速度将逐渐减小
      max_vel_y = std::max(std::min(max_vel_y, dist / sim_time_), min_vel_y);

      // if we use continous acceleration, we can sample the max velocity we can reach in sim_time_
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_time_);//这里考虑了当前速度情况下和加速度约束下，机器人的能到的最大速度（连续恒定加速度作用下）
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_time_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_time_);

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_time_);//同样这里考虑了当前速度情况下和加速度约束下，机器人的能到的最小速度（连续恒定加速度作用下）
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_time_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_time_);

    } else {
      //这里使用DWA算法, 最大的速度不超过加速度作用下的第一步的速度
      // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_period_);//sim_period_是模拟的间隔
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_period_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_period_);

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_period_);
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_period_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_period_);
    }

    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
    VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]);//速度迭代器，最小速度，最大速度和迭代的数量
    VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]);
    VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]);
    //二维是三重循环遍历所有情况
    for(; !x_it.isFinished(); x_it++) {
      vel_samp[0] = x_it.getVelocity();
      for(; !y_it.isFinished(); y_it++) {
        vel_samp[1] = y_it.getVelocity();
        for(; !th_it.isFinished(); th_it++) {
          vel_samp[2] = th_it.getVelocity();
          //ROS_DEBUG("Sample %f, %f, %f", vel_samp[0], vel_samp[1], vel_samp[2]);
          sample_params_.push_back(vel_samp);
        }
        th_it.reset(); //角度恢复到起始值
      }
      y_it.reset();//y轴的速度恢复到起始值
    }
  }
}

//设置轨迹生成的参数
void SimpleTrajectoryGenerator::setParameters(
    double sim_time,
    double sim_granularity,
    double angular_sim_granularity,
    bool use_dwa,
    double sim_period) {
  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;
  angular_sim_granularity_ = angular_sim_granularity;
  use_dwa_ = use_dwa;
  continued_acceleration_ = ! use_dwa_;
  sim_period_ = sim_period;
}

/**
 * Whether this generator can create more trajectories
 */
//检查是否还有轨迹没有读取
bool SimpleTrajectoryGenerator::hasMoreTrajectories() {
  return next_sample_index_ < sample_params_.size();
}

/**
 * Create and return the next sample trajectory
 */
//生成下一条轨迹
bool SimpleTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) {
  bool result = false;
  if (hasMoreTrajectories()) {
    if (generateTrajectory(
        pos_,
        vel_,
        sample_params_[next_sample_index_],
        comp_traj)) {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}


/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool SimpleTrajectoryGenerator::generateTrajectory(
      Eigen::Vector3f pos,/*当前机器人的位姿*/
      Eigen::Vector3f vel,/*当前机器人的速度*/
      Eigen::Vector3f sample_target_vel,/*期望的机器人采样速度*/
      base_local_planner::Trajectory& traj) {

  double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);//获取期望采样的平移速度，由x和y方向合成
  double eps = 1e-4;//偏差
  traj.cost_   = -1.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoints();

  // make sure that the robot would at least be moving with one of
  // the required minimum velocities for translation and rotation (if set)
  //如果期望采样的速度比配置的最小速度都小，则配置出错，返回
  if ((limits_->min_vel_trans >= 0 && vmag + eps < limits_->min_vel_trans) &&
      (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_vel_theta)) {
    return false;
  }
  // make sure we do not exceed max diagonal (x+y) translational velocity (if set)
  //同样，如果期望采样的速度比配置的最大速度都大，则配置出错，返回
  if (limits_->max_vel_trans >=0 && vmag - eps > limits_->max_vel_trans) {
    return false;
  }

  int num_steps;
  //如果配置了离散化，就采用模拟的总时间和模拟的间隔计算总的模拟步数
  if (discretize_by_time_) { //discretize_by_time_是否进行离散化
    num_steps = ceil(sim_time_ / sim_granularity_);//sim_time_是总的模拟时间，sim_granularity_是模拟的间隔
  }
  else {
    //compute the number of steps we must take along this trajectory to be "safe"
    //如果没有配置离散化，则利用机器人在当前速度下最大距离除以模拟时间间隔作为模拟步数
    double sim_time_distance = vmag * sim_time_; // the distance the robot would travel in sim_time if it did not change velocity
    double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_; // the angle the robot would rotate in sim_time
    num_steps =
        ceil(std::max(sim_time_distance / sim_granularity_,
            sim_time_angle    / angular_sim_granularity_));
  }

  if (num_steps == 0) {
    return false;
  }

  //compute a timestep
  //计算每次模拟的时间间隔
  double dt = sim_time_ / num_steps;
  traj.time_delta_ = dt;

  Eigen::Vector3f loop_vel;
  if (continued_acceleration_) {//如果配置了连续加速，则取第一次迭代的数据作为当前的速度
    // assuming the velocity of the first cycle is the one we want to store in the trajectory object
    loop_vel = computeNewVelocities(sample_target_vel, vel, limits_->getAccLimits(), dt);
    traj.xv_     = loop_vel[0];
    traj.yv_     = loop_vel[1];
    traj.thetav_ = loop_vel[2];
  }
  else {
    // assuming sample_vel is our target velocity within acc limits for one timestep
    loop_vel = sample_target_vel;
    traj.xv_     = sample_target_vel[0];
    traj.yv_     = sample_target_vel[1];
    traj.thetav_ = sample_target_vel[2];
  }

  //simulate the trajectory and check for collisions, updating costs along the way
  for (int i = 0; i < num_steps; ++i) {

    //add the point to the trajectory so we can draw it later if we want
    traj.addPoint(pos[0], pos[1], pos[2]);
     //一个速度点到一个速度点是否连续加速
    if (continued_acceleration_) {
      //calculate velocities
      loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
      //ROS_WARN_NAMED("Generator", "Flag: %d, Loop_Vel %f, %f, %f", continued_acceleration_, loop_vel[0], loop_vel[1], loop_vel[2]);
    }

    //update the position of the robot using the velocities passed in
    pos = computeNewPositions(pos, loop_vel, dt);

  } // end for simulation steps

  return true; // trajectory has at least one point
}
//根据速度计算下一个位置
Eigen::Vector3f SimpleTrajectoryGenerator::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;//因为这里加了0.5pi，这里cos（M_PI_2 + pos[2]）是等于-sin(pos[2])
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

/**
 * change vel using acceleration limits to converge towards sample_target-vel
 */
//考虑加速度计算下一个速度
Eigen::Vector3f SimpleTrajectoryGenerator::computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
    const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt) {
  Eigen::Vector3f new_vel = Eigen::Vector3f::Zero();
  for (int i = 0; i < 3; ++i) {
    if (vel[i] < sample_target_vel[i]) {
      new_vel[i] = std::min(double(sample_target_vel[i]), vel[i] + acclimits[i] * dt);
    }
    else {
      new_vel[i] = std::max(double(sample_target_vel[i]), vel[i] - acclimits[i] * dt);
    }
  }
  return new_vel;
}

} /* namespace base_local_planner */
