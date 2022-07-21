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
*   * Neither the name of the Willow Garage nor the names of its
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
#ifndef POINT_GRID_H_
#define POINT_GRID_H_
#include <vector>
#include <list>
#include <cfloat>
#include <geometry_msgs/Point.h>
#include <costmap_2d/observation.h>
#include <base_local_planner/world_model.h>

#include <sensor_msgs/PointCloud2.h>

namespace base_local_planner {
/**
   * @class PointGrid
   * @brief A class that implements the WorldModel interface to provide
   * free-space collision checks for the trajectory controller. This class
   * stores points binned into a grid and performs point-in-polygon checks when
   * necessary to determine the legality of a footprint at a given
   * position/orientation.
   */
//实现WorldModel接口的类，用于为轨迹控制器提供自由空间碰撞检查。此类将点存储到网格中，
//并在必要时执行多边形中的点检查，以确定给定位置/方向上足迹的合法性
class PointGrid : public WorldModel {
public:
  /**
       * @brief  Constuctor for a grid that stores points in the plane 构造一个在平面保存点的栅格地图
       * @param  width The width in meters of the grid //地图的宽度
       * @param  height The height in meters of the gird  //地图的高度
       * @param  resolution The resolution of the grid in meters/cell 地图的分辨率
       * @param  origin The origin of the bottom left corner of the grid 起点是从左下角开始 ? 左上角 ?
       * @param  max_z The maximum height for an obstacle to be added to the grid 加到栅格点的障碍物的高度
       * @param  obstacle_range The maximum distance for obstacles to be added to the grid 加到栅格地图的障碍物之间最大距离
       * @param  min_separation The minimum distance between points in the grid 栅格点之间最小的距离
       */
  PointGrid(double width, double height, double resolution, geometry_msgs::Point origin,
            double max_z, double obstacle_range, double min_separation);

  /**
       * @brief  Destructor for a point grid
       */
  virtual ~PointGrid(){}

  /**
       * @brief  Returns the points that lie within the cells contained in the specified range. Some of these points may be outside the range itself.
       * @param  lower_left The lower left corner of the range search
       * @param  upper_right The upper right corner of the range search
       * @param points A vector of pointers to lists of the relevant points
       */
  //返回保存在栅格地图中指定范围的所有点
  void getPointsInRange(const geometry_msgs::Point& lower_left, const geometry_msgs::Point& upper_right, std::vector< std::list<geometry_msgs::Point32>* >& points);

  /**
       * @brief  Checks if any points in the grid lie inside a convex footprint
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return Positive if all the points lie outside the footprint, negative otherwise
       */
  //检查是否栅格地图中一些点在机器人底盘位置
  virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
                               double inscribed_radius, double circumscribed_radius);

  using WorldModel::footprintCost;

  /**
       * @brief  Inserts observations from sensors into the point grid
       * @param footprint The footprint of the robot in its current location
       * @param observations The observations from various sensors
       * @param laser_scans The laser scans used to clear freespace (the point grid only uses the first scan which is assumed to be the base laser)
       */
  //根据实时获取的激光雷达数据动态更新地图
  void updateWorld(const std::vector<geometry_msgs::Point>& footprint,
                   const std::vector<costmap_2d::Observation>& observations, const std::vector<PlanarLaserScan>& laser_scans);

  /**
       * @brief  Convert from world coordinates to grid coordinates
       * @param  pt A point in world space
       * @param  gx The x coordinate of the corresponding grid cell to be set by the function
       * @param  gy The y coordinate of the corresponding grid cell to be set by the function
       * @return True if the conversion was successful, false otherwise
       */
    //将一个点从世界坐标转换到栅格地图的坐标
  inline bool gridCoords(geometry_msgs::Point pt, unsigned int& gx, unsigned int& gy) const {

    //是否世界坐标的位置小于起始位置（这里的世界坐标与实际机器人的运动坐标是有区别）
    if(pt.x < origin_.x || pt.y < origin_.y){
      gx = 0;
      gy = 0;
      return false;
    }
    gx = (int) ((pt.x - origin_.x)/resolution_);
    gy = (int) ((pt.y - origin_.y)/resolution_);

    if(gx >= width_ || gy >= height_){
      gx = 0;
      gy = 0;
      return false;
    }

    return true;
  }

  /**
       * @brief  Get the bounds in world coordinates of a cell in the point grid, assumes a legal cell when called
       * @param  gx The x coordinate of the grid cell
       * @param  gy The y coordinate of the grid cell
       * @param  lower_left The lower left bounds of the cell in world coordinates to be filled in
       * @param  upper_right The upper right bounds of the cell in world coordinates to be filled in
       */
  //获取指定栅格点在实际坐标系统的范围
  inline void getCellBounds(unsigned int gx, unsigned int gy, geometry_msgs::Point& lower_left, geometry_msgs::Point& upper_right) const {
    lower_left.x = gx * resolution_ + origin_.x;
    lower_left.y = gy * resolution_ + origin_.y;

    upper_right.x = lower_left.x + resolution_;
    upper_right.y = lower_left.y + resolution_;
  }


  /**
       * @brief  Compute the squared distance between two points
       * @param pt1 The first point
       * @param pt2 The second point
       * @return The squared distance between the two points
       */
  //计算两个点的欧式距离的平方
  inline double sq_distance(const geometry_msgs::Point32& pt1, const geometry_msgs::Point32& pt2){
    return (pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y);
  }

  /**
       * @brief  Convert from world coordinates to grid coordinates
       * @param  pt A point in world space
       * @param  gx The x coordinate of the corresponding grid cell to be set by the function
       * @param  gy The y coordinate of the corresponding grid cell to be set by the function
       * @return True if the conversion was successful, false otherwise
       */
  //将一个世界坐标的点转换到栅格坐标的点
  inline bool gridCoords(const geometry_msgs::Point32& pt, unsigned int& gx, unsigned int& gy) const {
    if(pt.x < origin_.x || pt.y < origin_.y){
      gx = 0;
      gy = 0;
      return false;
    }
    gx = (int) ((pt.x - origin_.x)/resolution_);
    gy = (int) ((pt.y - origin_.y)/resolution_);

    if(gx >= width_ || gy >= height_){
      gx = 0;
      gy = 0;
      return false;
    }

    return true;
  }

  /**
       * @brief  Converts cell coordinates to an index value that can be used to look up the correct grid cell
       * @param gx The x coordinate of the cell
       * @param gy The y coordinate of the cell
       * @return The index of the cell in the stored cell list
       */
  //获取指定栅格宽度、高度对应的栅格地图值的索引
  inline unsigned int gridIndex(unsigned int gx, unsigned int gy) const {
    /*
         * (0, 0) ---------------------- (width, 0)
         *  |                               |
         *  |                               |
         *  |                               |
         *  |                               |
         *  |                               |
         * (0, height) ----------------- (width, height)
         */
    return(gx + gy * width_);
  }

  /**
       * @brief  Check the orientation of a pt c with respect to the vector a->b
       * @param a The start point of the vector
       * @param b The end point of the vector
       * @param c The point to compute orientation for
       * @return orient(a, b, c) < 0 ----> Right, orient(a, b, c) > 0 ----> Left
       */
  //判断c点在a->b向量的哪一侧
  /*
   * |
   * |
   * |
   * .b
   * |    .c
   * |____.a_______
   */
    inline double orient(const geometry_msgs::Point& a, const geometry_msgs::Point& b, const geometry_msgs::Point32& c){
      double acx = a.x - c.x;
      double bcx = b.x - c.x;

      double acy = a.y - c.y;
      double bcy = b.y - c.y;
      return acx * bcy - acy * bcx;
    }

    /**
         * @brief  Check the orientation of a pt c with respect to the vector a->b
         * @param a The start point of the vector
         * @param b The end point of the vector
         * @param c The point to compute orientation for
         * @return orient(a, b, c) < 0 ----> Right, orient(a, b, c) > 0 ----> Left
         */
    template<typename T>
    inline double orient(const T& a, const T& b, const T& c){
      double acx = a.x - c.x;
      double bcx = b.x - c.x;
      double acy = a.y - c.y;
      double bcy = b.y - c.y;
      return acx * bcy - acy * bcx;
    }

    /**
         * @brief  Check if two line segmenst intersect
         * @param v1 The first point of the first segment
         * @param v2 The second point of the first segment
         * @param u1 The first point of the second segment
         * @param u2 The second point of the second segment
         * @return True if the segments intersect, false otherwise
         */
    //判断两条线段是否相交, 原理比较简单，一个线段的两个端点分别在另一个线段的两侧
    inline bool segIntersect(const geometry_msgs::Point32& v1, const geometry_msgs::Point32& v2,
                             const geometry_msgs::Point32& u1, const geometry_msgs::Point32& u2){
      return (orient(v1, v2, u1) * orient(v1, v2, u2) < 0) && (orient(u1, u2, v1) * orient(u1, u2, v2) < 0);
    }

    /**
         * @brief  Find the intersection point of two lines
         * @param v1 The first point of the first segment
         * @param v2 The second point of the first segment
         * @param u1 The first point of the second segment
         * @param u2 The second point of the second segment
         * @param result The point to be filled in
         */
    //获取两条线段的交点
    void intersectionPoint(const geometry_msgs::Point& v1, const geometry_msgs::Point& v2,
                           const geometry_msgs::Point& u1, const geometry_msgs::Point& u2,
                           geometry_msgs::Point& result);

    /**
         * @brief  Check if a point is in a polygon
         * @param pt The point to be checked
         * @param poly The polygon to check against
         * @return True if the point is in the polygon, false otherwise
         */
    //检查某个点是否在一个多边形内
    bool ptInPolygon(const geometry_msgs::Point32& pt, const std::vector<geometry_msgs::Point>& poly);

    /**
         * @brief  Insert a point into the point grid
         * @param pt The point to be inserted
         */
    //将一个点插入栅格地图
    void insert(const geometry_msgs::Point32& pt);

    /**
         * @brief  Find the distance between a point and its nearest neighbor in the grid
         * @param pt The point used for comparison
         * @return  The distance between the point passed in and its nearest neighbor in the point grid
         */
    //获取一个点最近邻居对应的栅格地图距离
    double nearestNeighborDistance(const geometry_msgs::Point32& pt);

    /**
         * @brief  Find the distance between a point and its nearest neighbor in a cell
         * @param pt The point used for comparison
         * @param gx The x coordinate of the cell
         * @param gy The y coordinate of the cell
         * @return  The distance between the point passed in and its nearest neighbor in the cell
         */
    //获取一个点在栅格地图中邻居的距离(该点对应栅格点的邻居距离)
    double getNearestInCell(const geometry_msgs::Point32& pt, unsigned int gx, unsigned int gy);

    /**
         * @brief  Removes points from the grid that lie within the polygon
         * @param poly A specification of the polygon to clear from the grid
         */
    //在栅格地图中移除某些点
    void removePointsInPolygon(const std::vector<geometry_msgs::Point> poly);

    /**
         * @brief  Removes points from the grid that lie within a laser scan
         * @param  laser_scan A specification of the laser scan to use for clearing
         */
    //同激光雷达的扫描数据清除栅格地图的点
    void removePointsInScanBoundry(const PlanarLaserScan& laser_scan);

    /**
         * @brief  Checks to see if a point is within a laser scan specification
         * @param  pt The point to check
         * @param  laser_scan The specification of the scan to check against
         * @return True if the point is contained within the scan, false otherwise
         */
    //检查某个点是否在激光雷达的扫描范围内
    bool ptInScan(const geometry_msgs::Point32& pt, const PlanarLaserScan& laser_scan);

    /**
         * @brief  Get the points in the point grid
         * @param  cloud The point cloud to insert the points into
         */
    //获取地图中所有的点
    void getPoints(sensor_msgs::PointCloud2& cloud);

  private:
    ///< @brief The resolution of the grid in meters/cell
    double resolution_;//地图的分辨率
    ///< @brief The origin point of the grid
    geometry_msgs::Point origin_;//栅格地图的起点
    ///< @brief The width of the grid in cells
    unsigned int width_;//栅格地图的宽度
    ///< @brief The height of the grid in cells
    unsigned int height_;//栅格地图的高度
    ///< @brief Storage for the cells in the grid
    std::vector< std::list<geometry_msgs::Point32> > cells_;//栅格地图中所有栅格值
    ///< @brief The height cutoff for adding points as obstacles
    double max_z_;//障碍物的高度
    ///< @brief The square distance at which we no longer add obstacles to the grid
    double sq_obstacle_range_;//不再添加障碍物到栅格地图的平方距离
    ///< @brief The minimum square distance required between points in the grid
    double sq_min_separation_;//栅格地图中点与点之间需要的最小距离
    ///< @brief The lists of points returned by a range search, made a member to save on memory allocation
    std::vector< std::list<geometry_msgs::Point32>* > points_;  //返回搜索某个访问的所有点
  };
  }
  #endif
