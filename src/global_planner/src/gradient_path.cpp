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
#include <global_planner/gradient_path.h>
#include <algorithm>
#include <stdio.h>
#include <global_planner/planner_core.h>

namespace global_planner {

GradientPath::GradientPath(PotentialCalculator* p_calc) :
        Traceback(p_calc), pathStep_(0.5) {  //这里默认路径的步进是0.5，这个参数应该要提供修改的接口
    gradx_ = grady_ = NULL;
}

GradientPath::~GradientPath() {
    //析构函数，释放分配的内存
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
}

void GradientPath::setSize(int xs, int ys) {
    Traceback::setSize(xs, ys);
    //释放分配的内存
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
    //根据指定大小分配内存
    gradx_ = new float[xs * ys];
    grady_ = new float[xs * ys];
}

bool GradientPath::getPath(float* potential, double start_x, double start_y, double goal_x, double goal_y, std::vector<std::pair<float, float> >& path) {

    std::pair<float, float> current;//临时变量，用于保存路径点

    int stc = getIndex(goal_x, goal_y);//获取目标点对应的索引

    // set up offset
    float dx = goal_x - (int)goal_x;//获取取整后的偏差x
    float dy = goal_y - (int)goal_y;//获取取整后的偏差y

    int ns = xs_ * ys_;//总的栅格数

    memset(gradx_, 0, ns * sizeof(float));//所有的x梯度值都置0
    memset(grady_, 0, ns * sizeof(float));//所有的y梯度值都置0

    int c = 0;//当前迭代次数，防止死循环
    while (c++ < ns * 4) {//这里的4需要提供修改的接口

        // check if near start
        double nx = stc % xs_ + dx;
        double ny = stc / xs_ + dy;
        //因为此处是从目标点往起始点回溯的，这里检测是否接近起始点
        //这里的0.5误差应提供修改的接口
        if (fabs(nx - start_x) < .5 && fabs(ny - start_y) < .5) {
            current.first = start_x;
            current.second = start_y;
            path.push_back(current);
            return true;//这里说明已经找到起点位置，已经完成整改路径回溯的过程
        }

        if (stc < xs_ || stc > xs_ * ys_ - xs_) // would be out of bounds //为什么是小于xs_和大于xs_ * ys_ - xs_, 因为下面会用到左右上下四个值
        {
            printf("[PathCalc] Out of bounds\n");
            return false;
        }

        current.first = nx;
        current.second = ny;

        //ROS_INFO("%d %d | %f %f ", stc%xs_, stc/xs_, dx, dy);

        path.push_back(current);//把路径点先放入容器

        bool oscillation_detected = false;//检查当前路径点附近是否振荡即来回运动

        int npath = path.size(); //获取当前路径点的个数

        //通过检测前一个和后一个路径点是否相同去判断是否振荡
        if (npath > 2 && path[npath - 1].first == path[npath - 3].first
                && path[npath - 1].second == path[npath - 3].second) {
            ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
            oscillation_detected = true;
        }


        int stcnx = stc + xs_;//当前栅格的下面栅格
        int stcpx = stc - xs_;//当前栅格的上面栅格

        // check for potentials at eight positions near cell检测附近的8个点去判断该路径点是否障碍物
        if (potential[stc] >= POT_HIGH || potential[stc + 1] >= POT_HIGH || potential[stc - 1] >= POT_HIGH
                || potential[stcnx] >= POT_HIGH || potential[stcnx + 1] >= POT_HIGH || potential[stcnx - 1] >= POT_HIGH
                || potential[stcpx] >= POT_HIGH || potential[stcpx + 1] >= POT_HIGH || potential[stcpx - 1] >= POT_HIGH
                || oscillation_detected) {
            ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potential[stc], (int) path.size());
            // check eight neighbors to find the lowest
            //从这八个点中找出最小的势场点
            int minc = stc;
            int minp = potential[stc];
            int st = stcpx - 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st = stc - 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st = stc + 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st = stcnx - 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            stc = minc;
            dx = 0;
            dy = 0;

            //ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
            //    potential[stc], path[npath-1].first, path[npath-1].second);

            if (potential[stc] >= POT_HIGH) { //如果最小势场都无穷大，说明这个路径点是不行的，这条路径是失败的
                ROS_DEBUG("[PathCalc] No path found, high potential");
                //savemap("navfn_highpot");
                return 0;
            }
        }

        // have a good gradient here
        //如果这八个点的梯度都不是无穷大，则说明这一点比较好, 可以进行梯度计算
        else {

            // get grad at four positions near cell
            gradCell(potential, stc);//当前点
            gradCell(potential, stc + 1);//当前点的右边点
            gradCell(potential, stcnx);//当前点的下面
            gradCell(potential, stcnx + 1);//当前点的下面的右边

            //通过插值计算梯度
            // get interpolated gradient
            float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
            float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
            float x = (1.0 - dy) * x1 + dy * x2; // interpolated x

            float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
            float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
            float y = (1.0 - dy) * y1 + dy * y2; // interpolated y

            // show gradients
            ROS_DEBUG(
                    "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n", gradx_[stc], grady_[stc], gradx_[stc+1], grady_[stc+1], gradx_[stcnx], grady_[stcnx], gradx_[stcnx+1], grady_[stcnx+1], x, y);

            // check for zero gradient, failed
            //正常情况梯度不会为零
            if (x == 0.0 && y == 0.0) {
                ROS_DEBUG("[PathCalc] Zero gradient");
                return 0;
            }

            // move in the right direction
            float ss = pathStep_ / hypot(x, y); //hypot = sqrt(X*X + Y*Y)

            dx += x * ss;
            dy += y * ss;

            // check for overflow
            if (dx > 1.0) {
                stc++;
                dx -= 1.0;
            }
            if (dx < -1.0) {
                stc--;
                dx += 1.0;
            }
            if (dy > 1.0) {
                stc += xs_;
                dy -= 1.0;
            }
            if (dy < -1.0) {
                stc -= xs_;
                dy += 1.0;
            }

        }

        //printf("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
        //         potential[stc], dx, dy, path[npath-1].first, path[npath-1].second);
    }

    return false;
}

/*
 int
 NavFn::calcPath(int n, int *st)
 {
 // set up start position at cell
 // st is always upper left corner for 4-point bilinear interpolation
 if (st == NULL) st = start;
 int stc = st[1]*nx + st[0];

 // go for <n> cycles at most
 for (int i=0; i<n; i++)
 {



 }

 //  return npath;            // out of cycles, return failure
 ROS_DEBUG("[PathCalc] No path found, path too long");
 //savemap("navfn_pathlong");
 return 0;            // out of cycles, return failure
 }
 */

//
// gradient calculations
//
// calculate gradient at a cell
// positive value are to the right and down
float GradientPath::gradCell(float* potential, int n) {

    if (gradx_[n] + grady_[n] > 0.0)    // check this cell
        return 1.0;

    if (n < xs_ || n > xs_ * ys_ - xs_)    // would be out of bounds
        return 0.0;

    float cv = potential[n]; //这一点的势场值
    float dx = 0.0;
    float dy = 0.0;

    // check for in an obstacle //如果这一点的势场值为无穷大，则为障碍物或者未遍历过
    if (cv >= POT_HIGH) {
        if (potential[n - 1] < POT_HIGH)
            dx = -lethal_cost_;//设置一个大的偏移量，远离障碍物点
        else if (potential[n + 1] < POT_HIGH)
            dx = lethal_cost_;//设置一个大的偏移量，远离障碍物点
        if (potential[n - xs_] < POT_HIGH)
            dy = -lethal_cost_;//设置一个大的偏移量，远离障碍物点
        else if (potential[n + xs_] < POT_HIGH)
            dy = lethal_cost_;//设置一个大的偏移量，远离障碍物点
    }

    else                // not in an obstacle
    {
        //梯度如何求：前后势场值的差
        // dx calc, average to sides
        if (potential[n - 1] < POT_HIGH)
            dx += potential[n - 1] - cv; //positive value are to the right and down这里为什么是左减右，期望算出的值正的指向右边和下面
        if (potential[n + 1] < POT_HIGH)
            dx += cv - potential[n + 1];

        // dy calc, average to sides
        if (potential[n - xs_] < POT_HIGH)
            dy += potential[n - xs_] - cv;
        if (potential[n + xs_] < POT_HIGH)
            dy += cv - potential[n + xs_];
    }

    // normalize
    float norm = hypot(dx, dy);//需要标准化，保证方向一致

    if (norm > 0) {
        norm = 1.0 / norm;//梯度的值不会超过1
        gradx_[n] = norm * dx;
        grady_[n] = norm * dy;//
    }
    return norm;
}

} //end namespace global_planner

