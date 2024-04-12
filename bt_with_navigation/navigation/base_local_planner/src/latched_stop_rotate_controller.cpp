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
  private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

  rotating_to_goal_ = false;
}

LatchedStopRotateController::~LatchedStopRotateController() {}


/**
 * returns true if we have passed the goal position.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 * Also goal orientation might not yet be true
 */
bool LatchedStopRotateController::isPositionReached(LocalPlannerUtil* planner_util,
    const geometry_msgs::PoseStamped& global_pose) {
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;

  //we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;

  //check to see if we've reached the goal position
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||
      base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
    xy_tolerance_latch_ = true;
    return true;
  }
  return false;
}


/**
 * 在运动规划之前，move_base先利用DWAPlannerROS::isGoalReached(内部利用LatchedStopRotateController::isGoalReached函数实现)
 * 来判断是否已经到达了目标点(即位置在xy_goal_tolerance以内，朝向在yaw_goal_tolerance以内,且速度小于
 * trans_stopped_velocity,rot_stopped_velocity),如果是则控制结束，即发送0速度，且复位move_base相关控制标志，并返回action成功的结果。
 * 否则，利用DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)计算局部规划速度。
 * returns true if we have passed the goal position and have reached goal orientation.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 */
bool LatchedStopRotateController::isGoalReached(LocalPlannerUtil* planner_util,
    OdometryHelperRos& odom_helper,
    const geometry_msgs::PoseStamped& global_pose) {
  //获取目标值容差
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;//位置容差
  double theta_stopped_vel = planner_util->getCurrentLimits().theta_stopped_vel;//角度停止速度
  double trans_stopped_vel = planner_util->getCurrentLimits().trans_stopped_vel;//位置停止速度

  //获取当前里程计信息，存储在base_odom变量中
  nav_msgs::Odometry base_odom;
  odom_helper.getOdom(base_odom);

  //获取全局规划的最后一个点作为目标位置，并存储在goal_pose变量中
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }
  //获取目标位置的x，y坐标
  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;
  //获取当前规划器的限制参数，存储在limits变量中
  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  //检查是否已经到达目标位置
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||
      base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
    //如果用户要求目标容差固定，并且xy_tolerance_latch_已经为true，或者当前位置与目标位置的距离小于等于xy_goal_tolerance，表示已经到目标位置
    if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_) {
      //如果用户要求目标容差固定，并且xy_tolerance_latch_为false，将其设置为true
      ROS_DEBUG("Goal position reached (check), stopping and turning in place");
      xy_tolerance_latch_ = true;
    }
    double goal_th = tf2::getYaw(goal_pose.pose.orientation);
    double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
    //检查是否达到了目标方向
    if (fabs(angle) <= limits.yaw_goal_tolerance) {
      //确保在返回成功之前我们真的停止了
      if (base_local_planner::stopped(base_odom, theta_stopped_vel, trans_stopped_vel)) {
        return true;
      }
    }
  }
  return false;
}

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
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, yaw),
                                  Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
                                  Eigen::Vector3f(vx, vy, vth));

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
    const geometry_msgs::PoseStamped& global_pose,  //机器人全局位姿
    const geometry_msgs::PoseStamped& robot_vel,  //机器人当前速度信息
    double goal_th, //目标朝向的角度
    geometry_msgs::Twist& cmd_vel,  //生成的速度指令将存储在这个参数中
    Eigen::Vector3f acc_lim,  //机器人线速度、角速度加速度限制
    double sim_period,  //仿真周期，用于进行运动预测
    base_local_planner::LocalPlannerLimits& limits, //本地规划器的限制参数
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) { //检查是否存在障碍物的函数
  setlocale(LC_ALL, "");
  ROS_INFO("机器人已到达x,y位姿容差值内，正在旋转方向至目标方向");
  // 1、首先，从全局位姿中获取当前偏航角度yaw，以及机器人当期的偏航角速度vel_yaw
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  // 2、将线速度指令cmd_vel.linear.x 和 cmd_vel.linear.y设置为0，因为该方法只关注旋转控制
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  // 3、计算当前偏航角与目标偏航角之间的角度差ang_diff，使用angles::shortest_angular_distance()函数获取最短角度差
  double ang_diff = angles::shortest_angular_distance(yaw, goal_th);
  // 限制旋转角速度的范围，取limits.min_vel_theta和limits.max_vel_theta之间的最小值，并将其限制在机器人角加速度的范围内。
  // 这样做是为了考虑机器人的加速度限制。
  double v_theta_samp = std::min(limits.max_vel_theta, std::max(limits.min_vel_theta, fabs(ang_diff)));

  //take the acceleration limits of the robot into account
  //考虑机器人的加速度限制
  double max_acc_vel = fabs(vel_yaw) + acc_lim[2] * sim_period;
  double min_acc_vel = fabs(vel_yaw) - acc_lim[2] * sim_period;

  v_theta_samp = std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

  //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
  //计算最大速度 max_speed_to_stop，即根据加速度限制下能够停止的最大速度，并将 v_theta_samp 限制在这个范围内。
  double max_speed_to_stop = sqrt(2 * acc_lim[2] * fabs(ang_diff));
  v_theta_samp = std::min(max_speed_to_stop, fabs(v_theta_samp));
  // 6、重新限制旋转角速度的范围，确保 v_theta_samp 在 limits.min_vel_theta 和 limits.max_vel_theta 之间。
  v_theta_samp = std::min(limits.max_vel_theta, std::max(limits.min_vel_theta, v_theta_samp));
  // 7、如果ang_diff为负值，则将v_theta_samp取反，以实现逆时针旋转
  if (ang_diff < 0) {
    ROS_INFO("角度为负值，取反，实现逆时针旋转");
    v_theta_samp = - v_theta_samp;
  }

  //we still want to lay down the footprint of the robot and check if the action is legal
  // 8、调用 obstacle_check() 函数，传入机器人的位置、速度和旋转角速度，并检查是否存在障碍物。返回值 valid_cmd 表示当前速度指令是否合法。
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, yaw),
      Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
      Eigen::Vector3f( 0.0, 0.0, v_theta_samp));
  // 9、如果速度指令合法，将 v_theta_samp 赋值给 cmd_vel.angular.z，表示机器人的旋转角速度。
  if (valid_cmd) {
    ROS_DEBUG_NAMED("dwa_local_planner", "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);
    cmd_vel.angular.z = v_theta_samp;
    ROS_INFO("成功生成旋转指令");
    // 10、返回true，表示成功生成旋转指令
    return true;
  }
  // 11、如果速度指令不合法，输出警告信息，并将 cmd_vel.angular.z设置为零，并返回false
  ROS_WARN("Rotation cmd in collision");
  cmd_vel.angular.z = 0.0;
  return false;
}


//机器人到达目标点后，开始计算对应的减速停止 or 旋转至目标朝向的速度指令
//场景：机器人最后一段行驶方向与目标朝向一样，则为前者。否则为后者
bool LatchedStopRotateController::computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel, //生成速度指令将被存储在这个参数中，以便传递给机器人
    Eigen::Vector3f acc_lim,  //机器人的线性和角速度加速度限制
    double sim_period,  //仿真周期，用于进行运动预测
    LocalPlannerUtil* planner_util, //局部规划器实用工具类
    OdometryHelperRos& odom_helper_,  //OdometryHelperRos类型对象，用于获取机器人的当前位置和速度信息
    const geometry_msgs::PoseStamped& global_pose,  //机器人全局位姿
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) { //检查是否存在障碍物
  setlocale(LC_ALL, "");
  geometry_msgs::PoseStamped goal_pose;
  //函数主要逻辑如下：
  //1、通过调用planner_util->getGoal(goal_pose)获取全局目标位姿goal_pose
  if ( ! planner_util->getGoal(goal_pose)) {
    //2、如果无法获取全局目标位姿，则输出错误信息并返回false
    ROS_ERROR("Could not get goal pose");
    return false;
  }

  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();
  //3、如果设置latch_xy_goal_tolerance_为true且xy_tolerance_latch_为false，则表示机器人已到达目标位置，开始原地旋转
  if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_ ) {
    ROS_INFO("已到达目标位姿, 停止前进并进行原地旋转");
    xy_tolerance_latch_ = true;
  }
  //4、检查是否达到了目标朝向姿态，计算当前机器人朝向与目标值之间的角度差angle，并与角度容差limits.yaw_goal_tolerance进行比较
  double goal_th = tf2::getYaw(goal_pose.pose.orientation);
  double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
  //5、如果角度差小于角度容差
  if (fabs(angle) <= limits.yaw_goal_tolerance) {
    //将速度指令置为0
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    //将rotating_to_goal_设置为false，表示已旋转到目标朝向
    rotating_to_goal_ = false;
    ROS_INFO("机器人已到达目标位姿，将速度控制指令设置为0");
  } 
  //6、否则。。。
  else {
    //获取机器人的速度信息和里程计信息
    ROS_DEBUG("Angle: %f Tolerance: %f", angle, limits.yaw_goal_tolerance);
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    nav_msgs::Odometry base_odom;
    odom_helper_.getOdom(base_odom);

    //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
    //7、如果机器人尚未停止且没有旋转到目标朝向，将根据机器人的加速度限制来停止机器人，调用stopWithAccLimits()函数实现
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
      //8、如果成功停止机器人，则将rotating_to_goal_设置为true，表示正在旋转到目标朝向
      rotating_to_goal_ = true;
      //9、如果机器人已停止，则调用rotateToGoal()函数实现机器人旋转到目标朝向
      if ( ! rotateToGoal(
          global_pose,
          robot_vel,
          goal_th,
          cmd_vel,
          acc_lim,
          sim_period,
          limits,
          obstacle_check)) {
        //10、如果旋转过程中出现错误，则输出错误信息并返回false
        ROS_INFO("Error when rotating.");
        return false;
      }
      ROS_DEBUG("Rotating...");
    }
  }
  //11、最后，返回true，表示成功计算并生成速度指令
  return true;

}


} /* namespace base_local_planner */
