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
#ifndef COSTMAP_2D_COSTMAP_LAYER_H_
#define COSTMAP_2D_COSTMAP_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

namespace costmap_2d
{
//这个主要是用距离传感器去实时更新栅格地图
class CostmapLayer : public Layer, public Costmap2D
{
public:
  CostmapLayer() : has_extra_bounds_(false),
    extra_min_x_(1e6), extra_max_x_(-1e6),
    extra_min_y_(1e6), extra_max_y_(-1e6) {}

  //是否是离散地图
  bool isDiscretized()
  {
    return true;
  }
  //调整地图的大小与其他的相匹配
  virtual void matchSize();

  //清除地图中指定区域的地图数据
  virtual void clearArea(int start_x, int start_y, int end_x, int end_y);

  /**
   * If an external source changes values in the costmap,
   * it should call this method with the area that it changed
   * to ensure that the costmap includes this region as well.
   * @param mx0 Minimum x value of the bounding box
   * @param my0 Minimum y value of the bounding box
   * @param mx1 Maximum x value of the bounding box
   * @param my1 Maximum y value of the bounding box
   */
  //如果其他地图层修改了某个地图区域的数据，这个地图也需要添加这部分数据，动态更新调整
  void addExtraBounds(double mx0, double my0, double mx1, double my1);

protected:
  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * TrueOverwrite means every value from this layer
   * is written into the master grid.
   */
  //更新当前地图的值更新主地图
  void updateWithTrueOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Overwrite means every valid value from this layer
   * is written into the master grid (does not copy NO_INFORMATION)
   */
  //这个函数没有利用本地无信息的栅格值更新主地图
  void updateWithOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Sets the new value to the maximum of the master_grid's value
   * and this layer's value. If the master value is NO_INFORMATION,
   * it is overwritten. If the layer's value is NO_INFORMATION,
   * the master value does not change.
   */
  //将本层的最大值更新主地图，也就是如果本层地图的值大于主地图，则更新主地图
  void updateWithMax(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Sets the new value to the sum of the master grid's value
   * and this layer's value. If the master value is NO_INFORMATION,
   * it is overwritten with the layer's value. If the layer's value
   * is NO_INFORMATION, then the master value does not change.
   *
   * If the sum value is larger than INSCRIBED_INFLATED_OBSTACLE,
   * the master value is set to (INSCRIBED_INFLATED_OBSTACLE - 1).
   */
  //通过将本层地图值和主地图值相加作为主地图的值，如果本层地图值信息是未知，则不更新
  void updateWithAddition(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /**
   * Updates the bounding box specified in the parameters to include
   * the location (x,y)
   *
   * @param x x-coordinate to include
   * @param y y-coordinate to include
   * @param min_x bounding box
   * @param min_y bounding box
   * @param max_x bounding box
   * @param max_y bounding box
   */
   //获取包含x和y的边界
  void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

  /*
   * Updates the bounding box specified in the parameters
   * to include the bounding box from the addExtraBounds
   * call. If addExtraBounds was not called, the method will do nothing.
   *
   * Should be called at the beginning of the updateBounds method
   *
   * @param min_x bounding box (input and output)
   * @param min_y bounding box (input and output)
   * @param max_x bounding box (input and output)
   * @param max_y bounding box (input and output)
   */
  //
  void useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y);

  bool has_extra_bounds_;

private:
  double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
};

}  // namespace costmap_2d
#endif  // COSTMAP_2D_COSTMAP_LAYER_H_
