#pragma once
#ifndef _MOVE_BASE_POINT_H_
#define _MOVE_BASE_POINT_H_

#include <behaviortree_ros/bt_action_node.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "behaviortree_base.h"

int execute_number = 0;

class MoveBasePoint: public RosActionNode<move_base_msgs::MoveBaseAction>
{
public:
  MoveBasePoint( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<move_base_msgs::MoveBaseAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("frame_id"),
      InputPort<float>("position_x"),
      InputPort<float>("position_y"),
      InputPort<float>("orientation_w"),
    };
  }

  bool sendGoal(GoalType& goal) override
  {
    goal.target_pose.header.stamp = ros::Time::now(); // 设置时间戳为当前时间
    getInput("frame_id", goal.target_pose.header.frame_id);
    getInput("position_x", goal.target_pose.pose.position.x);
    getInput("position_y", goal.target_pose.pose.position.y);
    getInput("orientation_w", goal.target_pose.pose.orientation.w);
    ROS_INFO("MoveBasePoint: sending request");
    ros::Duration xx(0.5);
    xx.sleep();
    return true;
  }

//当机器人到达第一个点的时候，它的actionlib::SimpleClientGoalState会变成SUCCEEDED，这里的判断逻辑是当为SUCCEEDED时行为树就会return SUCCESS。
//此时立马发送第二个点，机器人的当前状态仍为SUCCEEDED，那么行为树就会认为机器人已经到达第二个点了，此时机器人不会去到第二个点。
//解决方法：1、在发送点的位置加上一定时间的延时,给move_base处理目标点时间。
//2、在发点功能加上一个线程来执行发点任务。
  NodeStatus onResult( const ResultType& res) override
  {
    ROS_INFO("MoveBasePoint: result received");
    
    // if(action_client_->waitForResult())
    // {
      // std::cout<< "state:"<< action_client_->getState() <<std::endl;
      if(action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          execute_number++;
          std::cout<<"------------当前执行到第： "<<execute_number<<" 个点------------"<<std::endl;
          return NodeStatus::SUCCESS;
        }
    // }
    else{
      ROS_ERROR("MoveBasePoint replied something unexpected:");
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("MoveBasePoint request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      ROS_WARN("MoveBasePoint halted");
      BaseClass::halt();
    }
  }
    
};

#endif