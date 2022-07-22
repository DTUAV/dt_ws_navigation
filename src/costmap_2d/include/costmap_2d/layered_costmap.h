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
#ifndef COSTMAP_2D_LAYERED_COSTMAP_H_
#define COSTMAP_2D_LAYERED_COSTMAP_H_

#include <costmap_2d/cost_values.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <string>

namespace costmap_2d
{
class Layer;

/**
 * @class LayeredCostmap
 * @brief Instantiates different layer plugins and aggregates them into one score
 */
//实例化不同的层插件并将其聚合为一个代价//带有代价地图的地图层
class LayeredCostmap
{
public:
  /**
   * @brief  Constructor for a costmap
   */
  LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

  /**
   * @brief  Destructor
   */
  ~LayeredCostmap();

  /**
   * @brief  Update the underlying costmap with new data.
   * If you want to update the map outside of the update loop that runs, you can call this.
   */
  //根据机器人的位置更新地图
  void updateMap(double robot_x, double robot_y, double robot_yaw);

  //获取全局坐标系的名称
  std::string getGlobalFrameID() const
  {
    return global_frame_;
  }
  //调整地图的大小
  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
                 bool size_locked = false);

  //获取代价地图更新的区域
  void getUpdatedBounds(double& minx, double& miny, double& maxx, double& maxy)
  {
    minx = minx_;
    miny = miny_;
    maxx = maxx_;
    maxy = maxy_;
  }

  bool isCurrent();

  //获取代价地图
  Costmap2D* getCostmap()
  {
    return &costmap_;
  }
  //代价地图是否跟随机器人运动
  bool isRolling()
  {
    return rolling_window_;
  }

  //是否当前代价地图的默认值是没有信息
  bool isTrackingUnknown()
  {
    return costmap_.getDefaultValue() == costmap_2d::NO_INFORMATION;
  }

  //返回所有地图层的共享指针
  std::vector<boost::shared_ptr<Layer> >* getPlugins()
  {
    return &plugins_;
  }

  //添加一个地图层
  void addPlugin(boost::shared_ptr<Layer> plugin)
  {
    plugins_.push_back(plugin);
  }

  //该地图层是否固定大小
  bool isSizeLocked()
  {
    return size_locked_;
  }

  //获取地图的边界
  void getBounds(unsigned int* x0, unsigned int* xn, unsigned int* y0, unsigned int* yn)
  {
    *x0 = bx0_;
    *xn = bxn_;
    *y0 = by0_;
    *yn = byn_;
  }

  //是否已经初始化
  bool isInitialized()
  {
      return initialized_;
  }

  /** @brief Updates the stored footprint, updates the circumscribed
   * and inscribed radii, and calls onFootprintChanged() in all
   * layers. */
  //设置机器人底盘的位置
  void setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec);

  /** @brief Returns the latest footprint stored with setFootprint(). */
  //获取机器人底盘的位置数据
  const std::vector<geometry_msgs::Point>& getFootprint() { return footprint_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which just surrounds all points on the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  //获取机器人底盘外圆的半径
  double getCircumscribedRadius() { return circumscribed_radius_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which is just within all points and edges of the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  //获取机器人内圆的半径
  double getInscribedRadius() { return inscribed_radius_; }

private:
  Costmap2D costmap_;//代价地图
  std::string global_frame_;//全局坐标系
  //代价地图是否和跟随机器人滚动
  bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot

  bool current_;
  //更新的区域
  double minx_, miny_, maxx_, maxy_;
  //地图边界
  unsigned int bx0_, bxn_, by0_, byn_;

  std::vector<boost::shared_ptr<Layer> > plugins_;

  bool initialized_;//是否已经初始化
  bool size_locked_;//是否固定地图大小
  //机器人底盘的内接圆半径和外接圆半径
  double circumscribed_radius_, inscribed_radius_;
  //机器人底盘的位置点
  std::vector<geometry_msgs::Point> footprint_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_LAYERED_COSTMAP_H_
