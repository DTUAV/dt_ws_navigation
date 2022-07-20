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
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#ifndef ABSTRACT_LOCAL_PLANNER_ODOM_H_
#define ABSTRACT_LOCAL_PLANNER_ODOM_H_

#include <nav_core/base_local_planner.h>

#include <boost/thread.hpp>

#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/buffer.h>

#include <base_local_planner/local_planner_limits.h>


namespace base_local_planner {
//局部规划器的使用接口
/**
 * @class LocalPlannerUtil
 * @brief Helper class implementing infrastructure code many local planner implementations may need.
 */
class LocalPlannerUtil {

private:
  // things we get from move_base
  std::string name_; //局部规划器的名字
  std::string global_frame_;//全局坐标系的名称

  costmap_2d::Costmap2D* costmap_;//代价地图
  tf2_ros::Buffer* tf_; //坐标变换的缓冲区，存有当前系统的所有坐标变换关系

  std::vector<geometry_msgs::PoseStamped> global_plan_;//全局路径规划出来的路径点

  boost::mutex limits_configuration_mutex_;//线程互斥锁

  bool setup_;//标志位，是否启动
  LocalPlannerLimits default_limits_; //默认参数//机器人运动的平移速度、平移加速度、角速度、角加速度、制动速度、制动加速度等参数的集合
  LocalPlannerLimits limits_;//修改后的参数//机器人运动的平移速度、平移加速度、角速度、角加速度、制动速度、制动加速度等参数的集合
  bool initialized_;//是否初始化

public:

  /**
   * @brief  Callback to update the local planner's parameters
   */
  void reconfigureCB(LocalPlannerLimits &config, bool restore_defaults);//动态配置参数的回调函数

  LocalPlannerUtil() : initialized_(false) {}//构造函数

  ~LocalPlannerUtil() {  //析构函数
  }

  void initialize(tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2D* costmap,
                  std::string global_frame);//初始化，需要初始化tf缓冲区、代价地图、全局坐标系名称

  bool getGoal(geometry_msgs::PoseStamped& goal_pose);//获取目标位置

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);//设置全局规划的路径点
  //获取局部路径规划的点
  bool getLocalPlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan);
  //获取代价地图
  costmap_2d::Costmap2D* getCostmap();
  //获取当前使用的机器人运动参数的约束
  LocalPlannerLimits getCurrentLimits();
  //获取全局坐标系的名称
  std::string getGlobalFrame(){ return global_frame_; }
};
}

#endif /* ABSTRACT_LOCAL_PLANNER_ODOM_H_ */
