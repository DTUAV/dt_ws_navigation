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
#ifndef _POTENTIAL_CALCULATOR_H
#define _POTENTIAL_CALCULATOR_H

#include <algorithm>

namespace global_planner {
//这个类是计算势场
class PotentialCalculator {
    public:
        PotentialCalculator(int nx, int ny) {
            setSize(nx, ny);
        }

        virtual float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential=-1){//这里使用默认形参初值，在Dijkstra算法中使用是默认形参初值，而A*算法中应用的是无默认形参初值
          //获取周围四个栅格的最小势场值
          if(prev_potential < 0){
                // get min of neighbors
                float min_h = std::min( potential[n - 1], potential[n + 1] ),
                      min_v = std::min( potential[n - nx_], potential[n + nx_]);
                prev_potential = std::min(min_h, min_v);
            }

            return prev_potential + cost;//周围的最小势场加上当前的代价
        }

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        //设置代价地图的宽度x，高度y和总的栅格数
        virtual void setSize(int nx, int ny) {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny;
        } /**< sets or resets the size of the map */

    protected:
        //根据宽度和高度获取地图的索引
        inline int toIndex(int x, int y) {
            return x + nx_ * y;
        }
        /**< size of grid, in pixels */
        int nx_;//地图的宽度x
        int ny_;//地图的高度y
        int ns_;//地图的栅格总数
};

} //end namespace global_planner
#endif
