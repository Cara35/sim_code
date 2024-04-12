#pragma once
#ifndef _TAKE_PHOTO_H_
#define _TAKE_PHOTO_H_

#include <std_srvs/Trigger.h>
#include <behaviortree_ros/bt_service_node.h>
#include "behaviortree_base.h"

// 定义一个ROS服务行为节点，用于调用相机拍照服务
class TakePhoto: public RosServiceNode<std_srvs::Trigger>
{
public:
  TakePhoto( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<std_srvs::Trigger>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      OutputPort<std::string>("arm_result") };
  }

  void sendRequest(RequestType& request) override
  {
    ROS_INFO("TakePhoto: sending request");
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("TakePhoto: response received");
    if( rep.success == true)
    {
      setOutput<bool>("res_name", rep.success);  // key : value
      std::cout<<"res name is "<<rep.success<<std::endl;;
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_ERROR("TakePhoto replied something unexpected: %s", rep.success);
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("TakePhoto request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

private:
  std::string expected_result_;
};

#endif