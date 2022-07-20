#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
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
#define POT_HIGH 1.0e10        // unassigned cell potential
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>  //代价地图
#include <geometry_msgs/PoseStamped.h>  //机器人位姿、期望目标点
#include <geometry_msgs/Point.h>        //位置
#include <nav_msgs/Path.h>              //路径
#include <vector>                           //vector容器
#include <nav_core/base_global_planner.h>  //全局规划器的基类
#include <nav_msgs/GetPlan.h>              //服务，获取路径
#include <dynamic_reconfigure/server.h>    //动态参数配置的服务
#include <global_planner/potential_calculator.h>  //势场计算
#include <global_planner/expander.h>              //规划器基类
#include <global_planner/traceback.h>             //路径回溯的基类
#include <global_planner/orientation_filter.h>    //方向滤波器
#include <global_planner/GlobalPlannerConfig.h>   //全局规划器的动态配置服务

namespace global_planner {

class Expander;   //对类声明，全局规划器
class GridPath;  //对类声明，栅格路径

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the global_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the PlannerCore object
         *///默认构造函数
        GlobalPlanner();

        /**
         * @brief  Constructor for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        //name: 全局规划器的名字， costmap：代价地图，frame_id: 坐标系名称
        GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        ~GlobalPlanner();

        /**
         * @brief  Initialization function for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        //继承基类的虚函数
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        //继承基类的路径生成函数
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        //继承基类的路径生成函数，这里添加容许到达目标点的偏差
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief  Computes the full navigation function for the map given a point in the world to start from
         * @param world_point The point to use for seeding the navigation function
         * @return True if the navigation function was computed successfully, false otherwise
         */
        //给定一个点，计算整个地图的势场
        bool computePotential(const geometry_msgs::Point& world_point);

        /**
         * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
         * @param start_x
         * @param start_y
         * @param end_x
         * @param end_y
         * @param goal The goal pose to create a plan to
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        //从势场中获取规划的路径
        bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @return The navigation function's value at that point in the world
         */
        //获取某点的势场
        double getPointPotential(const geometry_msgs::Point& world_point);

        /**
         * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @return True if the navigation function is valid at that point in the world, false otherwise
         */
        //验证某点的势场是否有效
        bool validPointPotential(const geometry_msgs::Point& world_point);

        /**
         * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @param tolerance The tolerance on searching around the world_point specified
         * @return True if the navigation function is valid at that point in the world, false otherwise
         */
        //给出容忍度去验证某一点的势场
        bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);

        /**
         * @brief  Publish a path for visualization purposes
         */
        //发布生成的路径
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        //路径获取的服务响应, 通过请求获取路径
        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

    protected:

        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        costmap_2d::Costmap2D* costmap_; //代价地图
        std::string frame_id_; //坐标系的名称
        ros::Publisher plan_pub_;//地图路径的发布器
        bool initialized_;//是否已经初始化
        bool allow_unknown_;//地图是否全部未知

    private:
        void mapToWorld(double mx, double my, double& wx, double& wy);//坐标变换，地图到世界坐标
        bool worldToMap(double wx, double wy, double& mx, double& my);//坐标变换，世界到地图
        void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my); //清除机器人的栅格
        void publishPotential(float* potential);//发布地图的势场

        double planner_window_x_;//规划器的窗口大小x
        double planner_window_y_;//规划器的窗口大小y
        double default_tolerance_;//默认的距离目标点的容忍度

        boost::mutex mutex_;//线程互斥锁

        ros::ServiceServer make_plan_srv_;//请求地图的服务器

        PotentialCalculator* p_calc_; //计算势场的类指针
        Expander* planner_;//全局规划器类指针
        Traceback* path_maker_;//路径回溯类指针
        OrientationFilter* orientation_filter_;//方向滤波器类指针

        bool publish_potential_;//是否发布势场
        ros::Publisher potential_pub_;//地图势场的发布器
        int publish_scale_; //发布势场的缩放因子

        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value); //给地图添加轮廓

        float* potential_array_;//势场数据指针

        unsigned int start_x_;//起点x
        unsigned int start_y_;//起点y
        unsigned int end_x_;//目标点x
        unsigned int end_y_;//目标点y

        bool old_navfn_behavior_;//
        float convert_offset_;//

        bool outline_map_;//是否给地图添加轮廓

        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig> *dsrv_;//动态配置参数的服务器

        void reconfigureCB(global_planner::GlobalPlannerConfig &config, uint32_t level);//动态配置参数的回调函数

};

} //end namespace global_planner

#endif
