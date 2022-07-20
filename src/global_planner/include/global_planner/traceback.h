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
#ifndef _TRACEBACK_H
#define _TRACEBACK_H
#include<vector>
#include<global_planner/potential_calculator.h>

namespace global_planner {
//这是最后输出路径的基类，从规划器生成的势场中获取路径
class Traceback {
    public:
        Traceback(PotentialCalculator* p_calc) : p_calc_(p_calc) {}
        //虚函数，从规划器生成的势场中获取规划的路径, 这里的=0是表示空，由继承的子类实现
        virtual bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) = 0;

        //虚函数，设置势场的大小
        virtual void setSize(int xs, int ys) {
            xs_ = xs;
            ys_ = ys;
        }

        //这是一个内联函数，提高运行的效率，编译器会将函数体复制到函数的调用处
        inline int getIndex(int x, int y) {
            return x + y * xs_;
        }

        //类的接口函数，设置中性的代价基准
        void setLethalCost(unsigned char lethal_cost) {
            lethal_cost_ = lethal_cost;
        }
    protected:
        int xs_;//栅格地图的宽度x
        int ys_;//栅格地图的高度y
        unsigned char lethal_cost_;//中性的代价基准
        PotentialCalculator* p_calc_;//势场计算类对象
};

} //end namespace global_planner
#endif
