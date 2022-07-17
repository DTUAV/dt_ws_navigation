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
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include <carrot_planner/carrot_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//register this planner as a BaseGlobalPlanner plugin

/*
 * 注册这个规划器为全局规划器，如果是自定义全局规划器，这个需要添加
#define PLUGINLIB_EXPORT_CLASS(class_type, base_class_type) \
  CLASS_LOADER_REGISTER_CLASS(class_type, base_class_type);
  文件如下
#define CLASS_LOADER__REGISTER_MACRO_HPP_

#include <string>

#include "console_bridge/console.h"

#include "class_loader/class_loader_core.hpp"
#include "class_loader/console_bridge_compatibility.hpp"

#define CLASS_LOADER_REGISTER_CLASS_INTERNAL_WITH_MESSAGE(Derived, Base, UniqueID, Message) \
  namespace \
  { \
  struct ProxyExec ## UniqueID \
  { \
    typedef  Derived _derived; \
    typedef  Base _base; \
    ProxyExec ## UniqueID() \
    { \
      if (!std::string(Message).empty()) { \
        CONSOLE_BRIDGE_logInform("%s", Message);} \
      class_loader::class_loader_private::registerPlugin<_derived, _base>(#Derived, #Base); \
    } \
  }; \
  static ProxyExec ## UniqueID g_register_plugin_ ## UniqueID; \
  }  // namespace
#define CLASS_LOADER_REGISTER_CLASS_INTERNAL_HOP1_WITH_MESSAGE(Derived, Base, UniqueID, Message) \
  CLASS_LOADER_REGISTER_CLASS_INTERNAL_WITH_MESSAGE(Derived, Base, UniqueID, Message)
#define CLASS_LOADER_REGISTER_CLASS_WITH_MESSAGE(Derived, Base, Message) \
  CLASS_LOADER_REGISTER_CLASS_INTERNAL_HOP1_WITH_MESSAGE(Derived, Base, __COUNTER__, Message)
#define CLASS_LOADER_REGISTER_CLASS(Derived, Base) \
  CLASS_LOADER_REGISTER_CLASS_WITH_MESSAGE(Derived, Base, "")
#endif  // CLASS_LOADER__REGISTER_MACRO_HPP_
*/
PLUGINLIB_EXPORT_CLASS(carrot_planner::CarrotPlanner, nav_core::BaseGlobalPlanner)

namespace carrot_planner {

  CarrotPlanner::CarrotPlanner()
    : costmap_ros_(NULL), initialized_(false){}

  CarrotPlanner::CarrotPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);//在构造函数中调用基类的虚函数，此虚函数在这里被重写
  }
  //重写基类的虚函数
  void CarrotPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();//costmap_ros_是代价地图的一个接口

      ros::NodeHandle private_nh("~/" + name);//设置句柄及话题的名字前缀
      private_nh.param("step_size", step_size_, costmap_->getResolution());//如果没有设置步进，则采用栅格地图的分配率，也就是一个格子一个格子访问, 这个参数会影响路径规划的时间
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);//获取目标阈值,，小于阈值则认为机器人已经到达目标点
      world_model_ = new base_local_planner::CostmapModel(*costmap_); //设置世界地图的模型为代价地图模型

      initialized_ = true;
      ROS_INFO("The carrot planner has been initialized...");
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  //考虑机器人的形状去计算避障路径
  double CarrotPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //s1： 检查是否已经初始化
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }
    //s2: 从代价地图中获取机器人的底盘形状
    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3) // 机器人的底盘至少三个点描述
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);//调用地图模型中函数计算当前位置的代价
    return footprint_cost;
    /*================================================该函数定义=======================================================
     *
       * @brief  Subclass will implement this method to check a footprint at a given position and orientation for legality in the world
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return Positive if all the points lie outside the footprint, negative otherwise:
       *          -1 if footprint covers at least a lethal obstacle cell, or
       *          -2 if footprint covers at least a no-information cell, or
       *          -3 if footprint is partially or totally outside of the map


      virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius) = 0;


      double footprintCost(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius = 0.0, double circumscribed_radius=0.0){

        double cos_th = cos(theta);
        double sin_th = sin(theta);
        std::vector<geometry_msgs::Point> oriented_footprint;
        for(unsigned int i = 0; i < footprint_spec.size(); ++i){
          geometry_msgs::Point new_pt;
          new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
          new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
          oriented_footprint.push_back(new_pt);
        }

        geometry_msgs::Point robot_position;
        robot_position.x = x;
        robot_position.y = y;

        if(inscribed_radius==0.0){
          costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
        }

        return footprintCost(robot_position, oriented_footprint, inscribed_radius, circumscribed_radius);
      }
    */
  }

  /*
   * 规划路径
   * 输入： 起点、目标点、输出的轨迹
   * 返回值：是否成功规划路径
  */
  bool CarrotPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    //S1: 先检查是否已经初始化，如果没有初始化就报错
    if(!initialized_) {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }
    //S2: debug模式下，这条语句会输出指定的起点和目标点的信息
    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    //S3: 清除路径点的容器
    plan.clear();

    //S4: 获取当前环境的代价地图
    costmap_ = costmap_ros_->getCostmap();

    //S5: 检查目标点的坐标系是否与代价地图的坐标系一致，通过消息的head的frame_id判断
    //如果目标点坐标系统与代价地图的坐标系统不一致将输入错误，所以指定目标点的消息的话题的坐标系统一定和代价地图的一致
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }
    //S6: 获取目标、当前的航向、二维位置
    const double start_yaw = tf2::getYaw(start.pose.orientation);//从四元数中获取航向角（绕z轴的旋转）
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    //目标点和当前点的偏差
    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;

    /*
    * Normalizes the angle to be -M_PI circle to +M_PI circle
    * It takes and returns radians.
    */
    //将偏差的航向角转换到-pi到pi
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = goal_yaw;

    //S6: 顺着目标点和起点的方向，检查目标点是否有障碍物，如果有则将目标点移动到没有目标地方
    bool done = false;
    double scale = 1.0;
    double dScale = 0.01; //只做100步，如果100步成功，则退出当次的路径规划

    while(!done)
    {
      if(scale < 0)
      {
        target_x = start_x;
        target_y = start_y;
        target_yaw = start_yaw;
        ROS_WARN("The carrot planner could not find a valid plan for this goal");
        break;
      }
      //从目标点向起点延伸?
      target_x = start_x + scale * diff_x;
      target_y = start_y + scale * diff_y;
      target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);
      
      double footprint_cost = footprintCost(target_x, target_y, target_yaw);//检查当前位置是否存在障碍物，或者机器人处于这个状态是否安全
      if(footprint_cost >= 0) //有障碍物，这个代价会是负数
      {
        done = true;
      }
      scale -=dScale;
    }

    plan.push_back(start);
    geometry_msgs::PoseStamped new_goal = goal;
    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0, 0, target_yaw);//航向角转四元素

    new_goal.pose.position.x = target_x;
    new_goal.pose.position.y = target_y;

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    //这个容器保存两个元素：一个是起点的元素，另一个是新目标点的元素。如果目标点不处于障碍物的位置，则新目标点等于旧的目标点，否则等于顺着起点和目标点方向的第一个无障碍物的位置。
    plan.push_back(new_goal);
    return (done);
  }

}
