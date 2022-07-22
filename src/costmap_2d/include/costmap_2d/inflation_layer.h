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
#ifndef COSTMAP_2D_INFLATION_LAYER_H_
#define COSTMAP_2D_INFLATION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

namespace costmap_2d
{
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
//用于保存栅格地图中一个点及一个点附近最近障碍物的位置
class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
    index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int index_;//当前点对应的代价地图索引
  unsigned int x_, y_;//代价地图的坐标
  unsigned int src_x_, src_y_;//距离x_和y_最近的障碍物的位置
};

//膨胀地图层
class InflationLayer : public Layer
{
public:
  InflationLayer();

  //这里采用虚函数，因为基类的析构函数是设置为虚的
  virtual ~InflationLayer()
  {
    //释放所有动态开辟的内存
    deleteKernels();
    if (dsrv_)  //释放动态配置参数服务器的内存
      delete dsrv_;
    if (seen_)
      delete[] seen_;
  }
  //继承基类, 初始化函数
  virtual void onInitialize();
  //继承基类，更新地图的指定区域
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  //继承基类，更新代价
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  //是否离散化
  virtual bool isDiscretized()
  {
    return true;
  }
  //继承基类，保存地图的大小一致
  virtual void matchSize();

  //继承基类，重置地图
  virtual void reset() { onInitialize(); }

  /** @brief  Given a distance, compute a cost.
   * @param  distance The distance from an obstacle in cells
   * @return A cost value for the distance */
  //在障碍物点给定一个栅格距离计算代价值
  //采用指数函数计算代价
  virtual inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0)
      cost = LETHAL_OBSTACLE;//254
    else if (distance * resolution_ <= inscribed_radius_)
      cost = INSCRIBED_INFLATED_OBSTACLE;//253
    else
    {
      // make sure cost falls off by Euclidean distance
      double euclidean_distance = distance * resolution_;//将栅格距离转换为实际环境的距离
      double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));//采用指数函数计算代价，距离越近代价越大
      cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  /**
   * @brief Change the values of the inflation radius parameters
   * @param inflation_radius The new inflation radius
   * @param cost_scaling_factor The new weight
   */
  //设置膨胀层的参数，包括膨胀的半径和代价缩放因子
  void setInflationParameters(double inflation_radius, double cost_scaling_factor);

protected:
  //继承基类函数，机器人位置改变时会调用这个函数更新
  virtual void onFootprintChanged();
  boost::recursive_mutex* inflation_access_;//线程互斥锁

  double resolution_; //地图的分辨率
  double inflation_radius_;//膨胀的半径
  double inscribed_radius_;//内接圆半径
  double weight_;//计算代价的权重
  bool inflate_unknown_;//是否膨胀未知区域

private:
  /**
   * @brief  Lookup pre-computed distances
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  //查找预先计算的距离
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }

  /**
   * @brief  Lookup pre-computed costs
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  //查找预先计算的代价
  inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_costs_[dx][dy];
  }

  //计算两个容器，距离容器和代价容器
  void computeCaches();

  //删除距离容器和代价容器的值并释放内存空间
  void deleteKernels();

  //将给定的区域进行膨胀
  void inflate_area(int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);

  //将世界坐标系下的距离转换为栅格地图下的距离
  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  //
  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y);

  unsigned int cell_inflation_radius_;//栅格膨胀的半径
  unsigned int cached_cell_inflation_radius_;//缓存单元膨胀半径

  std::map<double, std::vector<CellData> > inflation_cells_;//膨胀地图层的数据

  bool* seen_;
  int seen_size_;

  unsigned char** cached_costs_;//代价容器的数据 //如果要获取碰撞地图层的代价就获取这个对象的数据 [i][j]:对应i和j位置的代价
  double** cached_distances_;//距离容器的数据
  //上一次更新的区域
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig> *dsrv_; //动态配置参数服务器

  void reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level);//动态配置参数的回调函数

  bool need_reinflation_; //下一次是否膨胀整个代价地图 ///< Indicates that the entire costmap should be reinflated next time around.
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_INFLATION_LAYER_H_
