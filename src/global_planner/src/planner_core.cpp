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
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

//register this planner as a BaseGlobalPlanner plugin
//以插件形式声明这个类为全局规划器
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

  //在地图的四周画上轮廓
  void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {

    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)//上面
      *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)//下面
      *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)//左边
      *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)//右边
      *pc = value;
  }

  GlobalPlanner::GlobalPlanner() :
    costmap_(NULL), initialized_(false), allow_unknown_(true),/*默认地图是未知的，允许在未知区域进行路径规划*/
    p_calc_(NULL), planner_(NULL), path_maker_(NULL), orientation_filter_(NULL),
    potential_array_(NULL) {
  }

  GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
    GlobalPlanner() {
    //initialize the planner
    initialize(name, costmap, frame_id);//初始化基类
  }

  GlobalPlanner::~GlobalPlanner() {
    if (p_calc_)
      delete p_calc_;
    if (planner_)
      delete planner_;
    if (path_maker_)
      delete path_maker_;
    if (dsrv_)
      delete dsrv_;
  }

  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }

  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
      ros::NodeHandle private_nh("~/" + name);
      costmap_ = costmap;
      frame_id_ = frame_id;

      unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

      private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);//是否配置非常精确
      if(!old_navfn_behavior_)
        convert_offset_ = 0.5;
      else
        convert_offset_ = 0.0;

      bool use_quadratic;
      private_nh.param("use_quadratic", use_quadratic, true);//配置势场的计算方式
      if (use_quadratic)
        p_calc_ = new QuadraticCalculator(cx, cy);
      else
        p_calc_ = new PotentialCalculator(cx, cy);

      bool use_dijkstra;
      private_nh.param("use_dijkstra", use_dijkstra, true);//默认是使用dijkstra算法
      if (use_dijkstra)
      {
        DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
        if(!old_navfn_behavior_)
          de->setPreciseStart(true);
        planner_ = de;
      }
      else
        planner_ = new AStarExpansion(p_calc_, cx, cy);

      bool use_grid_path;
      private_nh.param("use_grid_path", use_grid_path, false);//配置使用路径的类型,有两种路径供选择：栅格路径和梯度路径
      if (use_grid_path)
        path_maker_ = new GridPath(p_calc_);
      else
        path_maker_ = new GradientPath(p_calc_);

      orientation_filter_ = new OrientationFilter();

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);//规划路径发布器配置，话题名是“plan”
      potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);//势场发布配置

      private_nh.param("allow_unknown", allow_unknown_, true);//配置地图是否全未知
      planner_->setHasUnknown(allow_unknown_);

      private_nh.param("planner_window_x", planner_window_x_, 0.0);
      private_nh.param("planner_window_y", planner_window_y_, 0.0);

      private_nh.param("default_tolerance", default_tolerance_, 0.0);

      private_nh.param("publish_scale", publish_scale_, 100);
      private_nh.param("outline_map", outline_map_, true);

      make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);//规划路径请求服务

      //动态配置参数
      dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
      dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb = boost::bind(
            &GlobalPlanner::reconfigureCB, this, _1, _2);

      dsrv_->setCallback(cb);//设置动态配置参数的回调函数

      initialized_ = true;
    } else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }
  //动态配置参数的回调函数
  void GlobalPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level) {

    planner_->setLethalCost(config.lethal_cost);//设置极其危险的代价
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);//设置中性的代价
    planner_->setFactor(config.cost_factor);      //设置代价的计算参数
    publish_potential_ = config.publish_potential; //是否发布势场信息
    orientation_filter_->setMode(config.orientation_mode);//方向滤波器的模式
    orientation_filter_->setWindowSize(config.orientation_window_size);//方向窗口的大小
  }

  void GlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
      ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);//将指定的点代价设置为0
  }
  //请求规划路径的服务回调函数
  bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
  }
  //地图点转换为世界坐标的点
  void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx + convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my + convert_offset_) * costmap_->getResolution();
  }
  //世界坐标点转换为地图的点
  bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {

    //获取栅格地图的起点对应的世界坐标的起点
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();

    //获取栅格地图的分辨率
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)  //这个世界坐标点与实际机器人的坐标点是有区别的，这里向右向下是正，所以如果在栅格地图的点是不会小于起始点的
      return false;
    //世界坐标点转换为地图的点
    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;
    //检查转换后的点是否超出栅格地图的范围
    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
      return true;

    return false;
  }

  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
  }
  //核心函数，生成规划路径
  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    //S1:互斥锁，禁止其他改变当前的资源
    boost::mutex::scoped_lock lock(mutex_);

    //S2:检查是否已经初始化，这个是通过服务调用的，这个检测有必要
    if (!initialized_) {
      ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //S3:清空路径保存的容器
    //clear the plan, just in case
    plan.clear();

    //S4: 检查起点和目标点的坐标是否对应同一个坐标系
    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
      ROS_ERROR(
            "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    if (start.header.frame_id != global_frame) {
      ROS_ERROR(
            "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
      return false;
    }

    //S5: 获取地图下的起点和目标点
    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
      ROS_WARN(
            "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }

    if(old_navfn_behavior_) {
      start_x = start_x_i;
      start_y = start_y_i;
    }
    else {
      worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
      ROS_WARN_THROTTLE(1.0,
                        "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }
    if(old_navfn_behavior_){
      goal_x = goal_x_i;
      goal_y = goal_y_i;
    }
    else {
      worldToMap(wx, wy, goal_x, goal_y);
    }

    //S6: 清除代价地图中起点的代价
    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //S7: 配置规划器、势场计算
    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);//势场计算类
    planner_->setSize(nx, ny);//路径规划基类
    path_maker_->setSize(nx, ny);//全局规划器类
    potential_array_ = new float[nx * ny];//势场数据保存缓冲区
    //是否在地图四周上画轮廓
    if(outline_map_)
      outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    //S7: 根据起点、目标点、代价地图、迭代次数计算当前地图的势场
    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                     nx * ny * 2, potential_array_);

    //S8: ???
    if(!old_navfn_behavior_)
      planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);

    //S9: 如果配置发布势场数据，就发布势场数据
    if(publish_potential_)
      publishPotential(potential_array_);
    //S10: 获取路径
    if (found_legal) {
      //extract the plan
      if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
        //make sure the goal we push on has the same timestamp as the rest of the plan
        geometry_msgs::PoseStamped goal_copy = goal;
        goal_copy.header.stamp = ros::Time::now();
        plan.push_back(goal_copy);
      }
      else {
        ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
      }
    }
    else {
      ROS_ERROR("Failed to get a plan.");
    }
    //S11: 在路径点中添加旋转的方向（航向角）
    // add orientations if needed
    orientation_filter_->processPath(start, plan);

    //S12: 发布规划的路径
    //publish the plan for visualization purposes
    publishPlan(plan);
    //S13: 释放创建指针的内存
    delete[] potential_array_;
    return !plan.empty();
  }

//将路径信息保存到一个ros消息并发布
  void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    //S1: 判断是否初始化
    if (!initialized_) {
      ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan
    //S2: 创建路径消息
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    //S3: 将规划的路径信息保存到ros消息中
    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
      gui_path.poses[i] = path[i];
    }
    //S4: 调用发布器发布路径
    plan_pub_.publish(gui_path);
  }

  bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                           const geometry_msgs::PoseStamped& goal,
                                           std::vector<geometry_msgs::PoseStamped>& plan) {
    //S1: 检查是否初始化
    if (!initialized_) {
      ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;
    //S2: 从生成的势场中得到规划的路径
    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
      ROS_ERROR("NO PATH!");
      return false;
    }
    //S4: 将路径点保存到geometry_msgs::PoseStamped中
    ros::Time plan_time = ros::Time::now();

    //因为path保存的是从目标点到起点，所以这里反向获取
    for (int i = path.size() - 1; i >= 0; i--) {

      std::pair<float, float> point = path[i];
      //convert the plan to world coordinates
      //将栅格地图中的路径点转换到map坐标系下
      double world_x, world_y;
      mapToWorld(point.first, point.second, world_x, world_y);

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;    //这里是2D路径规划
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }
    if(old_navfn_behavior_){
      plan.push_back(goal);
    }
    return !plan.empty();
  }

//将势场转换为占据栅格地图发布出去
  void GlobalPlanner::publishPotential(float* potential)
  {
    int nx = costmap_->getSizeInCellsX();//栅格地图的宽度
    int ny = costmap_->getSizeInCellsY();//栅格地图的高度
    double resolution = costmap_->getResolution();//获取地图的分辨率
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);//

    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    //获取最大的势场值
    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
      float potential = potential_array_[i];
      if (potential < POT_HIGH) {
        if (potential > max) {
          max = potential;
        }
      }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
      if (potential_array_[i] >= POT_HIGH) {
        grid.data[i] = -1;
      } else
        grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
  }

} //end namespace global_planner
