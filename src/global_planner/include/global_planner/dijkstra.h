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
#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H

#define PRIORITYBUFSIZE 10000
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <global_planner/planner_core.h>
#include <global_planner/expander.h>

// inserting onto the priority blocks
#define push_cur(n)  { if (n >= 0 && n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ && currentEnd_ < PRIORITYBUFSIZE){ currentBuffer_[currentEnd_++] = n; pending_[n] = true; }}
#define push_next(n) { if (n >= 0 && n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ &&    nextEnd_ < PRIORITYBUFSIZE){    nextBuffer_[   nextEnd_++] = n; pending_[n] = true; }}
#define push_over(n) { if (n >= 0 && n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ &&    overEnd_ < PRIORITYBUFSIZE){    overBuffer_[   overEnd_++] = n; pending_[n] = true; }}

namespace global_planner {
class DijkstraExpansion : public Expander {
    public:
        DijkstraExpansion(PotentialCalculator* p_calc, int nx, int ny);
        ~DijkstraExpansion();
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                                float* potential);

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        //设置地图的大小和栅格总数，继承基类
        void setSize(int nx, int ny); /**< sets or resets the size of the map */

        //设置中性代价和优先级增量
        void setNeutralCost(unsigned char neutral_cost) {
            neutral_cost_ = neutral_cost;
            priorityIncrement_ = 2 * neutral_cost_;
        }
        //设置精度，到达目标点的精度
        void setPreciseStart(bool precise){ precise_ = precise; }
    private:

        /**
         * @brief  Updates the cell at index n
         * @param costs The costmap
         * @param potential The potential array in which we are calculating
         * @param n The index to update
         */
        //更新第n个栅格的代价和势场
        void updateCell(unsigned char* costs, float* potential, int n); /** updates the cell at index n */

        //根据代价地图获取代价
        float getCost(unsigned char* costs, int n) {
            float c = costs[n];
            if (c < lethal_cost_ - 1 || (unknown_ && c==255)) {
                c = c * factor_ + neutral_cost_; //其他点的计算公式
                if (c >= lethal_cost_)
                    c = lethal_cost_ - 1;
                return c;
            }
            return lethal_cost_;
        }

        /** block priority buffers */
        int *buffer1_, *buffer2_, *buffer3_; /**< storage buffers for priority blocks *///3个数据缓冲区
        //三个缓冲区，一个是当前的缓冲区：就是将节点放进去而没有取出的。一个是下一个路径点的缓冲区：当前这一步有好几个可选的方案。移出的缓冲区就是经过的点保存在这里、
        int *currentBuffer_, *nextBuffer_, *overBuffer_; /**< priority buffer block ptrs *///分别对应上面3个数据缓冲区的指针

        int currentEnd_, nextEnd_, overEnd_; /**< end points of arrays *///分别对应上面3个数据缓冲区的最后索引

        bool *pending_; /**< pending_ cells during propagation *///标志缓冲区，用于标志哪些栅格已经被访问过

        bool precise_;//是否需要精确

        /** block priority thresholds */
        float threshold_; /**< current threshold */

        float priorityIncrement_; /**< priority threshold increment */

};
} //end namespace global_planner
#endif
