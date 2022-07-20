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
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) { //调用基类的构造函数初始化
}

bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear();//清除vector容器内容

    int start_i = toIndex(start_x, start_y);//根据所给起点的宽度和高度转换成代价地图的索引

    queue_.push_back(Index(start_i, 0));//将起点的代价设置为0

    std::fill(potential, potential + ns_, POT_HIGH);//将地图中势场全部填充为无穷大

    potential[start_i] = 0; //将起点的势场设置为0

    int goal_i = toIndex(end_x, end_y);//根据所给目标点的宽度和高度转换成代价地图的索引

    int cycle = 0; //当前的迭代次数

    while (queue_.size() > 0 && cycle < cycles) {

        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());//将第0个元素和最后一个元素交换位置，并对前n-1个元素进行从小到大排序。也就是第0个元素的代价是最小的
        /*
         * make_heap()用于把一个可迭代容器变成一个堆，默认是大顶堆。它有三个参数。第一个参数是指向开始元素的迭代器，第二个参数是指向最末尾元素的迭代器，第三个参数是less<>()或是greater<>()，前者用于生成大顶堆，后者用于生成小顶堆，第三个参数默认情况下为less<>()，less<int>()用于生成大顶堆。
         * pop_heap()用于将堆的第零个元素与最后一个元素交换位置，然后针对前n - 1个元素调用make_heap()函数，它也有三个参数，参数意义与make_heap()相同，第三个参数应与make_heap时的第三个参数保持一致。
         * push_heap()用于把数据插入到堆中，它也有三个参数，其意义与make_heap()的相同，第三个参数应与make_heap时的第三个参数保持一致。
         * sort_heap()是将堆进行排序，排序后，序列将失去堆的特性（子节点的键值总是小于或大于它的父节点）。它也具有三个参数，参数意义与make_heap()相同，第三个参数应与make_heap时的第三个参数保持一致。大顶堆sort_heap()后是一个递增序列，小顶堆是一个递减序列。
        */
        queue_.pop_back(); //最小代价的元素已经保存在top了，这里将这个元素从容器中弹出

        int i = top.i; //取出最小元素的栅格地图索引

        if (i == goal_i)//如果当前的索引已经等于目标点的索引，说明已经找到路径，结束路径规划的迭代
            return true;

        add(costs, potential, potential[i], i + 1, end_x, end_y);//检查当前点的右边并计算代价

        add(costs, potential, potential[i], i - 1, end_x, end_y);//检查当前点的左边并计算代价

        add(costs, potential, potential[i], i + nx_, end_x, end_y);//检查当前点的下边并计算代价

        add(costs, potential, potential[i], i - nx_, end_x, end_y);//检查当前点的上边并计算代价

        cycle++;//迭代的次数加1
    }

    return false;
}
//
/*
  costs: 代价地图数据
  potential：势场数据
  prev_potential：上一个路径点的势场数据
  next_i: 下一个点的栅格地图索引
  end_x: 目标点x
  end_y: 目标点y
*/
void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y) {

    if (next_i < 0 || next_i >= ns_) //如果当前栅格地图的索引超出栅格地图，返回
        return;

    if (potential[next_i] < POT_HIGH)//如果这一点已经更新过，则返回
        return;
    //如果这一点是障碍物点，返回
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))//unknown_：是否允许在未知地图中规划路径
        return;

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential); //这里传入prev_potential, 所以这里实际的结果是costs[next_i] + neutral_cost_ + prev_potential

    int x = next_i % nx_;//获取当前索引对应的宽度x
    int y = next_i / nx_;//获取当前索引对应的高度y

    float distance = abs(end_x - x) + abs(end_y - y);//这里采用的是曼哈顿距离（街道距离），机器人只能往四个方向运动

    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));//potential[next_i] + distance * neutral_cost_)：将两种代价相加，作为路径点的代价

    std::push_heap(queue_.begin(), queue_.end(), greater1());//对容器的数据进行从小到大排序
}

} //end namespace global_planner
