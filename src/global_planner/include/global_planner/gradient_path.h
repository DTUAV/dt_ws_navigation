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
#ifndef _GRADIENT_PATH_H
#define _GRADIENT_PATH_H

#include <global_planner/traceback.h>
#include <math.h>
#include <algorithm>

namespace global_planner {

//获取梯度路径，对获取的路径进行插值
class GradientPath : public Traceback {
    public:
        GradientPath(PotentialCalculator* p_calc);
        ~GradientPath();

        //继承基类的虚函数，用于设置地图数据（梯度数据、势场数据）的大小
        void setSize(int xs, int ys);

        //
        // Path construction
        // Find gradient at array points, interpolate path
        // Use step size of pathStep, usually 0.5 pixel
        //
        // Some sanity checks:
        //  1. Stuck at same index position
        //  2. Doesn't get near goal
        //  3. Surrounded by high potentials
        //
        //这里对生成的路径做进一步的修正
        bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);

    private:
        //内联函数，根据给定x和y方向增量，获取距离n最近的点
        inline int getNearestPoint(int stc, float dx, float dy) {

            int pt = stc + (int)round(dx) + (int)(xs_ * round(dy));

            return std::max(0, std::min(xs_ * ys_ - 1, pt));//保证该点的索引不超过地图的边界
        }

        float gradCell(float* potential, int n); //获取栅格地图中第n索引的梯度，根据规划器构建的势场求解

        //地图的梯度数据
        float *gradx_;
        float *grady_; /**< gradient arrays, size of potential array */

        //地图点的增量
        float pathStep_; /**< step size for following gradient */
};

} //end namespace global_planner
#endif
