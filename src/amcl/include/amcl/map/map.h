/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map.h 1713 2003-08-23 04:03:43Z inspectorg $
 **************************************************************************/

#ifndef MAP_H
#define MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
struct _rtk_fig_t;

  
// Limits
#define MAP_WIFI_MAX_LEVELS 8

  //定义图元
// Description for a single map cell.用于描述地图中栅格的属性：当前栅格的状态和与占据状态的栅格的最小距离,用于描述当前的状态和附近环境的状态
typedef struct
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  int occ_state;

  // Distance to the nearest occupied cell
  double occ_dist;

  // Wifi levels
  //int wifi_levels[MAP_WIFI_MAX_LEVELS];

} map_cell_t;

//定义地图
// Description for a map
typedef struct
{
  // Map origin; the map is a viewport onto a conceptual larger map.
  double origin_x, origin_y;
  
  // Map scale (m/cell)
  double scale;

  // Map dimensions (number of cells)
  int size_x, size_y;
  
  // The map data, stored as a grid
  map_cell_t *cells;

  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_occ_dist;//???这个参数不太明白
  
} map_t;



/**************************************************************************
 * Basic map functions
 **************************************************************************/

// Create a new (empty) map //创建一个地图并返回地图的地址
map_t *map_alloc(void);

// Destroy a map
void map_free(map_t *map);//删除一个地图并且释放地图的内存

// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy, double oa);//获取地图(ox,oy)处的地图信息,oa是什么参数

// Load an occupancy map
int map_load_occ(map_t *map, const char *filename, double scale, int negate);//从文件中加载一个占据栅格地图,scale：地图的缩放  negate：是否翻转地图


// Load a wifi signal strength map
//int map_load_wifi(map_t *map, const char *filename, int index);

// Update the cspace distances
void map_update_cspace(map_t *map, double max_occ_dist);//????


/**************************************************************************
 * Range functions
 **************************************************************************/

// Extract a single range reading from the map
double map_calc_range(map_t *map, double ox, double oy, double oa, double max_range);//从地图中随机采取一个范围


/**************************************************************************
 * GUI/diagnostic functions
 **************************************************************************/

// Draw the occupancy grid
void map_draw_occ(map_t *map, struct _rtk_fig_t *fig);

// Draw the cspace map
void map_draw_cspace(map_t *map, struct _rtk_fig_t *fig);

// Draw a wifi map
void map_draw_wifi(map_t *map, struct _rtk_fig_t *fig, int index);


/**************************************************************************
 * Map manipulation macros
 **************************************************************************/

// Convert from map index to world coords//地图是以中心为原点//将地图坐标转换为世界坐标系统位置坐标
#define MAP_WXGX(map, i) (map->origin_x + ((i) - map->size_x / 2) * map->scale)
#define MAP_WYGY(map, j) (map->origin_y + ((j) - map->size_y / 2) * map->scale)

// Convert from world coords to map coords//将世界坐标转换为地图坐标
#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5) + map->size_y / 2)

// Test to see if the given map coords lie within the absolute map bounds.//检查当前的i和j是否在地图中
#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

// Compute the cell index for the given map coords.//获取地图的索引
#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)

#ifdef __cplusplus
}
#endif

#endif
