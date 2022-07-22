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
#ifndef COSTMAP_2D_COSTMAP_2D_H_
#define COSTMAP_2D_COSTMAP_2D_H_

#include <vector>
#include <queue>
#include <geometry_msgs/Point.h>
#include <boost/thread.hpp>

namespace costmap_2d
{

// convenient for storing x/y point pairs
//地图位置数据类，用于描述地图中位置
struct MapLocation
{
  unsigned int x;
  unsigned int y;
};

/**
 * @class Costmap2D
 * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
 */
class Costmap2D
{
  friend class CostmapTester;  // Need this for gtest to work correctly
public:
  /**
   * @brief  Constructor for a costmap
   * @param  cells_size_x The x size of the map in cells  地图的宽度
   * @param  cells_size_y The y size of the map in cells  地图的高度
   * @param  resolution The resolution of the map in meters/cell 地图的分辨率 每个栅格代表多少米
   * @param  origin_x The x origin of the map 地图的起点x，这是全局坐标系
   * @param  origin_y The y origin of the map 地图的起点y，这是全局坐标系
   * @param  default_value Default Value 地图中数据的默认值
   */
  //要构造一个二维的代价地图需要提供地图的宽度、高度、分辨率、地图的起点和默认值
  Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
            double origin_x, double origin_y, unsigned char default_value = 0);

  /**
   * @brief  Copy constructor for a costmap, creates a copy efficiently
   * @param map The costmap to copy
   */
  //复制构造函数
  Costmap2D(const Costmap2D& map);

  /**
   * @brief  Overloaded assignment operator
   * @param  map The costmap to copy
   * @return A reference to the map after the copy has finished
   */
  //赋值构造函数
  Costmap2D& operator=(const Costmap2D& map);

  /**
   * @brief  Turn this costmap into a copy of a window of a costmap passed in
   * @param  map The costmap to copy
   * @param win_origin_x The x origin (lower left corner) for the window to copy, in meters
   * @param win_origin_y The y origin (lower left corner) for the window to copy, in meters
   * @param win_size_x The x size of the window, in meters
   * @param win_size_y The y size of the window, in meters
   */
  //根据指定起点和大小复制一个代价地图一部分
  bool copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x,
                         double win_size_y);

  /**
   * @brief  Default constructor
   */
  Costmap2D();

  /**
   * @brief  Destructor
   */
  virtual ~Costmap2D();

  /**
   * @brief  Get the cost of a cell in the costmap
   * @param mx The x coordinate of the cell
   * @param my The y coordinate of the cell
   * @return The cost of the cell
   */
  //获取指定栅格位置的代价
  unsigned char getCost(unsigned int mx, unsigned int my) const;

  /**
   * @brief  Set the cost of a cell in the costmap
   * @param mx The x coordinate of the cell
   * @param my The y coordinate of the cell
   * @param cost The cost to set the cell to
   */
  //设置指定栅格的代价
  void setCost(unsigned int mx, unsigned int my, unsigned char cost);

  /**
   * @brief  Convert from map coordinates to world coordinates
   * @param  mx The x map coordinate
   * @param  my The y map coordinate
   * @param  wx Will be set to the associated world x coordinate
   * @param  wy Will be set to the associated world y coordinate
   */
  //将一个地图坐标转换为世界坐标
  void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

  /**
   * @brief  Convert from world coordinates to map coordinates
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @return True if the conversion was successful (legal bounds) false otherwise
   */
  //将一个世界坐标转换为地图坐标
  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;

  /**
   * @brief  Convert from world coordinates to map coordinates without checking for legal bounds
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @note   The returned map coordinates <b>are not guaranteed to lie within the map.</b>
   */
  //不进行边界检查的世界坐标到地图坐标的转换
  void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const;

  /**
   * @brief  Convert from world coordinates to map coordinates, constraining results to legal bounds.
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @note   The returned map coordinates are guaranteed to lie within the map.
   */
  //从世界坐标转换为地图坐标，将结果约束到合法边界
  void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const;

  /**
   * @brief  Given two map coordinates... compute the associated index
   * @param mx The x coordinate
   * @param my The y coordinate
   * @return The associated index
   */
  //根据指定代价地图的x和y获取索引
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  {
    return my * size_x_ + mx;
  }

  /**
   * @brief  Given an index... compute the associated map coordinates
   * @param  index The index
   * @param  mx Will be set to the x coordinate
   * @param  my Will be set to the y coordinate
   */
  //将索引转换为代价地图的x和y
  inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
  {
    my = index / size_x_;
    mx = index - (my * size_x_);
  }

  /**
   * @brief  Will return a pointer to the underlying unsigned char array used as the costmap
   * @return A pointer to the underlying unsigned char array storing cost values
   */
  //将返回一个指针，指向用作costmap的底层无符号字符数组
  unsigned char* getCharMap() const;

  /**
   * @brief  Accessor for the x size of the costmap in cells
   * @return The x size of the costmap
   */
  //获取代价地图宽度(多少个栅格)
  unsigned int getSizeInCellsX() const;

  /**
   * @brief  Accessor for the y size of the costmap in cells
   * @return The y size of the costmap
   */
  //获取代价地图的高度
  unsigned int getSizeInCellsY() const;

  /**
   * @brief  Accessor for the x size of the costmap in meters
   * @return The x size of the costmap (returns the centerpoint of the last legal cell in the map)
   */
  //获取代价地图的宽度（多少米）
  double getSizeInMetersX() const;

  /**
   * @brief  Accessor for the y size of the costmap in meters
   * @return The y size of the costmap (returns the centerpoint of the last legal cell in the map)
   */
  //获取代价地图的高度（多少米）
  double getSizeInMetersY() const;

  /**
   * @brief  Accessor for the x origin of the costmap
   * @return The x origin of the costmap
   */
  //获取代价地图的起点x
  double getOriginX() const;

  /**
   * @brief  Accessor for the y origin of the costmap
   * @return The y origin of the costmap
   */
  //获取代价地图的起点y
  double getOriginY() const;

  /**
   * @brief  Accessor for the resolution of the costmap
   * @return The resolution of the costmap
   */
  //获取代价地图的分辨率
  double getResolution() const;

  //设置地图的默认值
  void setDefaultValue(unsigned char c)
  {
    default_value_ = c;
  }
  //获取地图设置的默认值
  unsigned char getDefaultValue()
  {
    return default_value_;
  }

  /**
   * @brief  Sets the cost of a convex polygon to a desired value
   * @param polygon The polygon to perform the operation on
   * @param cost_value The value to set costs to
   * @return True if the polygon was filled... false if it could not be filled
   */
  //将凸多边形的代价设置为所需值
  bool setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value);

  /**
   * @brief  Get the map cells that make up the outline of a polygon
   * @param polygon The polygon in map coordinates to rasterize
   * @param polygon_cells Will be set to the cells contained in the outline of the polygon
   */
  //在代价地图中绘制多边形的轮廓
  void polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);

  /**
   * @brief  Get the map cells that fill a convex polygon
   * @param polygon The polygon in map coordinates to rasterize
   * @param polygon_cells Will be set to the cells that fill the polygon
   */
  //获取填充凸多边形的栅格单元
  void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);

  /**
   * @brief  Move the origin of the costmap to a new location.... keeping data when it can
   * @param  new_origin_x The x coordinate of the new origin
   * @param  new_origin_y The y coordinate of the new origin
   */
  //更新地图的起点
  virtual void updateOrigin(double new_origin_x, double new_origin_y);

  /**
   * @brief  Save the costmap out to a pgm file
   * @param file_name The name of the file to save
   */
  //保存地图
  bool saveMap(std::string file_name);

  //调整地图的大小
  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                 double origin_y);

  //重置地图
  void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);

  /**
   * @brief  Given distance in the world... convert it to cells
   * @param  world_dist The world distance
   * @return The equivalent cell distance
   */
  //将世界坐标下的距离转换为栅格地图下的栅格距离
  unsigned int cellDistance(double world_dist);

  // Provide a typedef to ease future code maintenance
  typedef boost::recursive_mutex mutex_t;//线程互斥锁

  mutex_t* getMutex()
  {
    return access_;
  }

protected://继承的子类能够访问
  /**
   * @brief  Copy a region of a source map into a destination map
   * @param  source_map The source map
   * @param sm_lower_left_x The lower left x point of the source map to start the copy
   * @param sm_lower_left_y The lower left y point of the source map to start the copy
   * @param sm_size_x The x size of the source map
   * @param  dest_map The destination map
   * @param dm_lower_left_x The lower left x point of the destination map to start the copy
   * @param dm_lower_left_y The lower left y point of the destination map to start the copy
   * @param dm_size_x The x size of the destination map
   * @param region_size_x The x size of the region to copy
   * @param region_size_y The y size of the region to copy
   */
  //将源地图的一个区域复制到一个目标地图
  template<typename data_type>
  void copyMapRegion(data_type* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                     unsigned int sm_size_x, data_type* dest_map, unsigned int dm_lower_left_x,
                     unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                     unsigned int region_size_y)
  {
    // we'll first need to compute the starting points for each map
    data_type* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
    data_type* dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

    // now, we'll copy the source map into the destination map
    for (unsigned int i = 0; i < region_size_y; ++i)
    {
      memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
      sm_index += sm_size_x;
      dm_index += dm_size_x;
    }
  }

  /**
   * @brief  Deletes the costmap, static_map, and markers data structures
   */
  //删除代价地图、静态地图和标志
  virtual void deleteMaps();

  /**
   * @brief  Resets the costmap and static_map to be unknown space
   */
  //将代价地图和静态地图全部设置为未知区域
  virtual void resetMaps();

  /**
   * @brief  Initializes the costmap, static_map, and markers data structures
   * @param size_x The x size to use for map initialization
   * @param size_y The y size to use for map initialization
   */
  //初始化代价地图、静态地图和标志
  virtual void initMaps(unsigned int size_x, unsigned int size_y);

  /**
   * @brief  Raytrace a line and apply some action at each step
   * @param  at The action to take... a functor
   * @param  x0 The starting x coordinate
   * @param  y0 The starting y coordinate
   * @param  x1 The ending x coordinate
   * @param  y1 The ending y coordinate
   * @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
   */
  //光线跟踪
  template<class ActionType>
  inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                           unsigned int max_length = UINT_MAX)
  {
    int dx = x1 - x0;
    int dy = y1 - y0;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * size_x_;

    unsigned int offset = y0 * size_x_ + x0;

    // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = hypot(dx, dy);
    double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

    // if x is dominant
    if (abs_dx >= abs_dy)
    {
      int error_y = abs_dx / 2;
      bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
      return;
    }

    // otherwise y is dominant
    int error_x = abs_dy / 2;
    bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
  }

private:
  /**
   * @brief  A 2D implementation of Bresenham's raytracing algorithm... applies an action at each step
   */
  template<class ActionType>
  inline void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                          int offset_b, unsigned int offset, unsigned int max_length)
  {
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i)
    {
      at(offset);
      offset += offset_a;
      error_b += abs_db;
      if ((unsigned int)error_b >= abs_da)
      {
        offset += offset_b;
        error_b -= abs_da;
      }
    }
    at(offset);
  }

  //符号函数，小于0为-1，大于0为1
  inline int sign(int x)
  {
    return x > 0 ? 1.0 : -1.0;
  }

  mutex_t* access_;
protected:
  unsigned int size_x_;
  unsigned int size_y_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  unsigned char* costmap_;
  unsigned char default_value_;

  //在地图中做标志
  class MarkCell
  {
  public:
    MarkCell(unsigned char* costmap, unsigned char value) :
      costmap_(costmap), value_(value)
    {
    }
    inline void operator()(unsigned int offset)
    {
      costmap_[offset] = value_;
    }
  private:
    unsigned char* costmap_;
    unsigned char value_;
  };

  //在地图中绘制轮廓
  class PolygonOutlineCells
  {
  public:
    PolygonOutlineCells(const Costmap2D& costmap, const unsigned char* char_map, std::vector<MapLocation>& cells) :
      costmap_(costmap), char_map_(char_map), cells_(cells)
    {
    }

    // just push the relevant cells back onto the list
    inline void operator()(unsigned int offset)
    {
      MapLocation loc;
      costmap_.indexToCells(offset, loc.x, loc.y);
      cells_.push_back(loc);
    }

  private:
    const Costmap2D& costmap_;
    const unsigned char* char_map_;
    std::vector<MapLocation>& cells_;
  };
};
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_H
