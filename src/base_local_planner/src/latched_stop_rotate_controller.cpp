/*
 * latched_stop_rotate_controller.cpp
 *
 *  Created on: Apr 16, 2012
 *      Author: tkruse
 */

#include <base_local_planner/latched_stop_rotate_controller.h>

#include <cmath>

#include <Eigen/Core>

#include <angles/angles.h>
#include <nav_msgs/Odometry.h>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/local_planner_limits.h>

#include <tf2/utils.h>

namespace base_local_planner {

LatchedStopRotateController::LatchedStopRotateController(const std::string& name) {
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);//是否在目标容忍度内停止机器

  rotating_to_goal_ = false;//当前机器人航向角未到设置的目标
}

LatchedStopRotateController::~LatchedStopRotateController() {}


/**
 * returns true if we have passed the goal position.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 * Also goal orientation might not yet be true
 */
bool LatchedStopRotateController::isPositionReached(LocalPlannerUtil* planner_util, const geometry_msgs::PoseStamped& global_pose) {

  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;//获取到的目标点的容忍度

  //we assume the global goal is the last point in the global plan
  //获取当前的目标点
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;

  //check to see if we've reached the goal position
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) || base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {

    xy_tolerance_latch_ = true;//如果到的目标点就锁住机器人，禁止机器人运动
    return true;
  }
  return false;
}


/**
 * returns true if we have passed the goal position and have reached goal orientation.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 */
bool LatchedStopRotateController::isGoalReached(LocalPlannerUtil* planner_util, OdometryHelperRos& odom_helper, const geometry_msgs::PoseStamped& global_pose) {

  //获取目标点的容忍度、制动角速度和线速度
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;
  double theta_stopped_vel = planner_util->getCurrentLimits().theta_stopped_vel;
  double trans_stopped_vel = planner_util->getCurrentLimits().trans_stopped_vel;

  //copy over the odometry information
  //获取当前的里程计信息
  nav_msgs::Odometry base_odom;
  odom_helper.getOdom(base_odom);

  //we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;

  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  //check to see if we've reached the goal position
  //先检查位置是否到达指定点，再检查航向角
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) || base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
    //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //just rotate in place
    if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_) {
      ROS_DEBUG("Goal position reached (check), stopping and turning in place");
      xy_tolerance_latch_ = true;
    }

    double goal_th = tf2::getYaw(goal_pose.pose.orientation);
    double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
    //check to see if the goal orientation has been reached
    if (fabs(angle) <= limits.yaw_goal_tolerance) {
      //make sure that we're actually stopped before returning success
      //如果位置和航向角都到指定点，则停止机器人的运动
      if (base_local_planner::stopped(base_odom, theta_stopped_vel, trans_stopped_vel)) {
        return true;
      }
    }
  }
  return false;
}

//考虑加速度约束的制动
bool LatchedStopRotateController::stopWithAccLimits(const geometry_msgs::PoseStamped& global_pose,
                                                    const geometry_msgs::PoseStamped& robot_vel,
                                                    geometry_msgs::Twist& cmd_vel,
                                                    Eigen::Vector3f acc_lim,
                                                    double sim_period,
                                                    boost::function<bool (Eigen::Vector3f pos,
                                                                          Eigen::Vector3f vel,
                                                                          Eigen::Vector3f vel_samples)> obstacle_check) {

  //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
  //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
  double vx = sign(robot_vel.pose.position.x) * std::max(0.0, (fabs(robot_vel.pose.position.x) - acc_lim[0] * sim_period));
  double vy = sign(robot_vel.pose.position.y) * std::max(0.0, (fabs(robot_vel.pose.position.y) - acc_lim[1] * sim_period));

  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim[2] * sim_period));

  //we do want to check whether or not the command is valid
  double yaw = tf2::getYaw(global_pose.pose.orientation);

  //检查获取的速度指令是否有效
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, yaw),
                                  Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
                                  Eigen::Vector3f(vx, vy, vth));

  //有效就保存，否则就置零，保持机器人静止
  //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
  if(valid_cmd){
    ROS_DEBUG_NAMED("latched_stop_rotate", "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = vth;
    return true;
  }
  ROS_WARN("Stopping cmd in collision");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  return false;
}

bool LatchedStopRotateController::rotateToGoal(
    const geometry_msgs::PoseStamped& global_pose,
    const geometry_msgs::PoseStamped& robot_vel,
    double goal_th,
    geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    base_local_planner::LocalPlannerLimits& limits,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) {
  //获取目标航向角
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  //获取当前的航向角速度
  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);

  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  //计算角度偏差
  double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

  //角度的偏差就是角速度了，这里对这个偏差进行限幅
  double v_theta_samp = std::min(limits.max_vel_theta, std::max(limits.min_vel_theta, fabs(ang_diff)));

  //take the acceleration limits of the robot into account
  //获取最大角加速度作用下角速度的最大和最小值 acc_lim[0]->x  acc_lim[1]->y  acc_lim[2]->yaw
  double max_acc_vel = fabs(vel_yaw) + acc_lim[2] * sim_period;
  double min_acc_vel = fabs(vel_yaw) - acc_lim[2] * sim_period;

  //最大最小值和最小最大值
  v_theta_samp = std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

  //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
  //需要考虑设置的速度能够到达指定的姿态
  double max_speed_to_stop = sqrt(2 * acc_lim[2] * fabs(ang_diff));
  v_theta_samp = std::min(max_speed_to_stop, fabs(v_theta_samp));

  v_theta_samp = std::min(limits.max_vel_theta, std::max(limits.min_vel_theta, v_theta_samp));

  if (ang_diff < 0) {
    v_theta_samp = - v_theta_samp; //根据偏差，设置方向
  }

  //we still want to lay down the footprint of the robot and check if the action is legal
  //判断当前的指令是否合法
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, yaw),
                                  Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
                                  Eigen::Vector3f( 0.0, 0.0, v_theta_samp));

  if (valid_cmd) {
    ROS_DEBUG_NAMED("dwa_local_planner", "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);
    cmd_vel.angular.z = v_theta_samp;
    return true;
  }
  ROS_WARN("Rotation cmd in collision");
  cmd_vel.angular.z = 0.0;
  return false;

}

//主要的函数，调用前面几个函数实现机器人的制动
bool LatchedStopRotateController::computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel,
                                                                    Eigen::Vector3f acc_lim,
                                                                    double sim_period,
                                                                    LocalPlannerUtil* planner_util,
                                                                    OdometryHelperRos& odom_helper_,
                                                                    const geometry_msgs::PoseStamped& global_pose,
                                                                    boost::function<bool (Eigen::Vector3f pos,
                                                                                          Eigen::Vector3f vel,
                                                                                          Eigen::Vector3f vel_samples)> obstacle_check) {
  //we assume the global goal is the last point in the global plan
  //获取目标点
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    ROS_ERROR("Could not get goal pose");
    return false;
  }
  //获取局部路径规划的约束参数
  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
  //just rotate in place
  //判断是否已经处于目标点
  if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_ ) {
    ROS_INFO("Goal position reached, stopping and turning in place");
    xy_tolerance_latch_ = true;
  }
  //check to see if the goal orientation has been reached
  double goal_th = tf2::getYaw(goal_pose.pose.orientation);
  double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
  if (fabs(angle) <= limits.yaw_goal_tolerance) {
    //set the velocity command to zero
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    rotating_to_goal_ = false;
  } else {
    ROS_DEBUG("Angle: %f Tolerance: %f", angle, limits.yaw_goal_tolerance);
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    nav_msgs::Odometry base_odom;
    odom_helper_.getOdom(base_odom);

    //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
    if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, limits.theta_stopped_vel, limits.trans_stopped_vel)) {
      if ( ! stopWithAccLimits(
             global_pose,
             robot_vel,
             cmd_vel,
             acc_lim,
             sim_period,
             obstacle_check)) {
        ROS_INFO("Error when stopping.");
        return false;
      }
      ROS_DEBUG("Stopping...");
    }
    //if we're stopped... then we want to rotate to goal
    else {
      //set this so that we know its OK to be moving
      rotating_to_goal_ = true;
      if ( ! rotateToGoal(
             global_pose,
             robot_vel,
             goal_th,
             cmd_vel,
             acc_lim,
             sim_period,
             limits,
             obstacle_check)) {
        ROS_INFO("Error when rotating.");
        return false;
      }
      ROS_DEBUG("Rotating...");
    }
  }

  return true;

}


} /* namespace base_local_planner */
