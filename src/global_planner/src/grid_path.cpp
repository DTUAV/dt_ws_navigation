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
#include <global_planner/grid_path.h>
#include <algorithm>
#include <stdio.h>
namespace global_planner {
/*
  potential: 势场数据
  start_x: 起点x
  start_y: 起点y
  end_x: 目标点x
  end_y: 目标点y
  path：保存路径点(路径是倒过来的，第1个元素是目标点)
*/
bool GridPath::getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) {

    std::pair<float, float> current;//这个作为路径点的临时变量

    //从目标点往起始点回溯路径，因为这里使用的是势场不是代价，在起点的势场是最低的
    current.first = end_x;
    current.second = end_y;

    int start_index = getIndex(start_x, start_y);//获取起点对应的索引，作为遍历结果的判断条件

    path.push_back(current);//先将目标点压入

    int c = 0;//作为遍历退出条件，当遍历很久时退出

    int ns = xs_ * ys_;//整个栅格的总数
    
    while (getIndex(current.first, current.second) != start_index) {//如果当前的索引不等于起始点索引，继续回溯路径

        float min_val = 1e10;//设置最小的代价为无穷大
        int min_x = 0, min_y = 0;//势场最小的栅格点
        //在一个点的上中下，左中右找最小势场的点
        for (int xd = -1; xd <= 1; xd++) {
            for (int yd = -1; yd <= 1; yd++) {
                if (xd == 0 && yd == 0)
                    continue;//跳过本身的那个点
                int x = current.first + xd;//当前宽度x
                int y = current.second + yd;//当前高度y
                int index = getIndex(x, y);//获取当前宽度和高度对应的索引
                //找该点附近最小的势场的点
                if (potential[index] < min_val) {
                    min_val = potential[index];
                    min_x = x;
                    min_y = y;
                }
            }
        }
        //找到地图的起点，这一点为不可到达点，说明路径回溯失败，返回
        if (min_x == 0 && min_y == 0)
            return false;
        //保存路径点
        current.first = min_x;
        current.second = min_y;
        path.push_back(current);

        //为了保证程序不陷入死循环，这里的参数4是需要调整的
        if(c++>ns*4){
            return false;
        }
    }

    return true;
}

} //end namespace global_planner

