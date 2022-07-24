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
#ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
#define COSTMAP_2D_COSTMAP_2D_ROS_H_

#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.hpp>
#include <tf2/LinearMath/Transform.h>

//获取配置参数
class SuperValue : public XmlRpc::XmlRpcValue
{
public:
  void setStruct(XmlRpc::XmlRpcValue::ValueStruct* a)
  {
    _type = TypeStruct;
    _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
  }
  void setArray(XmlRpc::XmlRpcValue::ValueArray* a)
  {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

namespace costmap_2d
{

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. */
class Costmap2DROS
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap2DROS(const std::string &name, tf2_ros::Buffer& tf);
  ~Costmap2DROS();

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  //启动代价地图，包括初始化、订阅传感器数据等
  void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  //停止更新代价地图
  void stop();

  /**
   * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
   */
  //暂停更新代价地图
  void pause();

  /**
   * @brief  Resumes costmap updates
   */
  //继续更新代价地图
  void resume();

  //更新代价地图数据
  void updateMap();

  /**
   * @brief Reset each individual layer
   */
  //重置每个地图层
  void resetLayers();

  /** @brief Same as getLayeredCostmap()->isCurrent(). */
  bool isCurrent()
    {
      return layered_costmap_->isCurrent();
    }

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
   * @return True if the pose was set successfully, false otherwise
   */
  //获取机器人当前的位姿
  bool getRobotPose(geometry_msgs::PoseStamped& global_pose) const;

  /** @brief Returns costmap name */
  //获取代价地图的名称
  std::string getName() const
    {
      return name_;
    }

  /** @brief Returns the delay in transform (tf) data that is tolerable in seconds */
  //获取坐标变换可容忍延迟的时间
  double getTransformTolerance() const
    {
      return transform_tolerance_;
    }

  /** @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   *
   * Same as calling getLayeredCostmap()->getCostmap(). */
  //获取代价地图
  Costmap2D* getCostmap()
    {
      return layered_costmap_->getCostmap();
    }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  //获取代价地图的坐标系名称
  std::string getGlobalFrameID()
    {
      return global_frame_;
    }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  //获取机器人机身坐标系的名称
  std::string getBaseFrameID()
    {
      return robot_base_frame_;
    }
  //获取代价地图层管理器，其管理碰撞层、静态层等
  LayeredCostmap* getLayeredCostmap()
    {
      return layered_costmap_;
    }

  /** @brief Returns the current padded footprint as a geometry_msgs::Polygon. */
  //获取机器人当前底盘的形状
  geometry_msgs::Polygon getRobotFootprintPolygon()
  {
    return costmap_2d::toPolygon(padded_footprint_);
  }

  /** @brief Return the current footprint of the robot as a vector of points.
   *
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  //获取机器人底盘位置数据（填充）
  std::vector<geometry_msgs::Point> getRobotFootprint()
  {
    return padded_footprint_;
  }

  /** @brief Return the current unpadded footprint of the robot as a vector of points.
   *
   * This is the raw version of the footprint without padding.
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  //获取机器人底盘位置数据 无填充，原始数据
  std::vector<geometry_msgs::Point> getUnpaddedRobotFootprint()
  {
    return unpadded_footprint_;
  }

  /**
   * @brief  Build the oriented footprint of the robot at the robot's current pose
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  //在机器人的当前位姿下构建机器人的定向足迹
  void getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const;

  /** @brief Set the footprint of the robot to be the given set of
   * points, padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  //通过给定的填充点构建机器人底盘位置数据
  void setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points);

  /** @brief Set the footprint of the robot to be the given polygon,
   * padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  //根据给定的几何形状构建机器人的底盘位置数据
  void setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint);

protected:
  LayeredCostmap* layered_costmap_;//代价地图的层管理器
  std::string name_;               //代价地图的名称
  tf2_ros::Buffer& tf_;  ///< @brief Used for transforming point clouds
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double transform_tolerance_;  ///< timeout before transform errors

private:
  /** @brief Set the footprint from the new_config object.
   *
   * If the values of footprint and robot_radius are the same in
   * new_config and old_config, nothing is changed. */
  //从新的配置数据中设置机器人底盘位置数据
  void readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                               const costmap_2d::Costmap2DConfig &old_config);
  //加载旧的参数
  void loadOldParameters(ros::NodeHandle& nh);
  //请求旧的参数数据
  void warnForOldParameters(ros::NodeHandle& nh);
  //检查旧的参数数据
  void checkOldParam(ros::NodeHandle& nh, const std::string &param_name);
  //从基类中复制参数数据
  void copyParentParameters(const std::string& plugin_name, const std::string& plugin_type, ros::NodeHandle& nh);
  //动态参数配置回调函数
  void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);
  //运动的回调函数
  void movementCB(const ros::TimerEvent &event);
  //地图更新的主线程
  void mapUpdateLoop(double frequency);

  bool map_update_thread_shutdown_;//是否关闭地图更新的主线程
  bool stop_updates_, initialized_, stopped_, robot_stopped_;//停止地图更新、初始化、停止、停止机器人运动
  boost::thread* map_update_thread_;  ///< @brief A thread for updating the map//地图更新的线程
  ros::Timer timer_;//定时器
  ros::Time last_publish_;//上一次发布的时间
  ros::Duration publish_cycle;//发布的周期
  pluginlib::ClassLoader<Layer> plugin_loader_;//各个地图层的插件导入器
  geometry_msgs::PoseStamped old_pose_;//上一帧机器人的位姿
  Costmap2DPublisher* publisher_;//代价地图发布器
  dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;//动态更新参数的服务器

  boost::recursive_mutex configuration_mutex_;//线程互斥锁

  ros::Subscriber footprint_sub_;//机器人底盘位置的订阅器
  ros::Publisher footprint_pub_;//机器人底盘数据的发布器
  std::vector<geometry_msgs::Point> unpadded_footprint_;//未填充的机器人底盘数据
  std::vector<geometry_msgs::Point> padded_footprint_;//填充的机器人底盘数据
  float footprint_padding_;
  costmap_2d::Costmap2DConfig old_config_;//旧的配置参数
};
// class Costmap2DROS
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_ROS_H
