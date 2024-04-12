/*********************************************************************
dwa_local_planner ::DWAPlannerROS对象是dwa_local_planner::DWAPlanner对象的包装器，将其功能公开为C++ ROS 包装器。
它在初始化时指定的 ROS 名称空间（假定为此处的名称）内运行。
它遵循nav_core包中的nav_core::BaseLocalPlanner接口。
*********************************************************************/
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
//下面三个也涉及到DWA算法实现
#include <nav_core/base_local_planner.h>  //继承了这个文件的所有接口
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <dwa_local_planner/dwa_planner.h>

namespace dwa_local_planner {
  /**
   * @class DWAPlannerROS
   * @brief DWAPlanner 的 ROS 包装器，它遵循 BaseLocalPlanner 接口，可用作move_base的插件。
   *        DWAPlannerROS继承了nav_core::BaseLocalPlanner类
   */
  class DWAPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  DWAPlannerROS包装器的构造函数。
       */
      DWAPlannerROS();

      /**
       * @brief  构建 ros 包装器。
       * @param name 给这个轨迹规划器实例的名称
       * @param tf 指向转换侦听器的指针
       * @param costmap 用于将成本分配给轨迹的成本图
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~DWAPlannerROS();

      /**
       * @brief  设置控制器遵循的计划
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  判断是否到达目标点
       * @return 已经到达为True，否则为false
       */
      bool isGoalReached();

       /**
       * @brief  给定机器人的当前位置、方向和速度，计算发送到机器人底盘的速度控制命令。
       * @param cmd_vel 将填充要传递给机器人底座的速度命令
       * @return 如果找到有效轨迹则为真，否则为假
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
      
      /**
       * @brief  给定机器人的当前位置、方向和速度，使用DWA计算速度命令以发送到机器人底盘。
       * @param cmd_vel 将填充要传递给机器人底座的速度命令
       * @return 如果找到有效轨迹则为真，否则为假
       */
      bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);


      bool isInitialized() {
        return initialized_;
      }

    private:
      //基于动态重新配置更新本地规划器参数的回调
      void reconfigureCB(DWAPlannerConfig &config, uint32_t level);

      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      tf2_ros::Buffer* tf_; ///< @brief 用于变换点云

      // for visualisation, publishers of global and local plan
      //用于可视化，发布全局和局部规划
      ros::Publisher g_plan_pub_, l_plan_pub_;

      base_local_planner::LocalPlannerUtil planner_util_;

      //声明一个dp_共享指针，该指针指向类型为DWAPlanner对象
      boost::shared_ptr<DWAPlanner> dp_; //轨迹控制器

      costmap_2d::Costmap2DROS* costmap_ros_;

      dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;

      dwa_local_planner::DWAPlannerConfig default_config_;

      bool setup_;

      geometry_msgs::PoseStamped current_pose_;

      base_local_planner::LatchedStopRotateController latchedStopRotateController_;


      bool initialized_;

      base_local_planner::OdometryHelperRos odom_helper_;

      std::string odom_topic_;
  };
};
#endif
