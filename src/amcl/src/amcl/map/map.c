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
 * CVS: $Id: map.c 1713 2003-08-23 04:03:43Z inspectorg $
**************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "amcl/map/map.h"


// Create a new map
map_t *map_alloc(void)
{
  map_t *map;//定义一个指针
  map = (map_t*) malloc(sizeof(map_t));//分配内存
  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;
  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;
  // Allocate storage for main map
  map->cells = (map_cell_t*) NULL;
  return map;
}


// Destroy a map
void map_free(map_t *map)
{
  free(map->cells);
  free(map);
  return;
}


// Get the cell at the given point//给定世界坐标的二维坐标获取栅格地图中指定的栅格值
map_cell_t *map_get_cell(map_t *map, double ox, double oy, double oa)//oa的参数没有用到//只是二维，oa是三维的高度
{
  int i, j;
  map_cell_t *cell;
  i = MAP_GXWX(map, ox);//获取对应ox的地图中栅格的索引
  j = MAP_GYWY(map, oy);//获取对应oy的地图中栅格的索引
  
  if (!MAP_VALID(map, i, j))
    return NULL;
  //map->cells: 首地址
  cell = map->cells + MAP_INDEX(map, i, j);//二维数组的地址
  return cell;
}

