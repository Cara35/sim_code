#pragma once
#ifndef _TAKE_PHOTO_H_
#define _TAKE_PHOTO_H_

#include <turtlesim/Spawn.h>
#include <behaviortree_ros/bt_service_node.h>
#include "behaviortree_base.h"


// 调用TurtleSpawn服务
class TurtleSpawn: public RosServiceNode<turtlesim::Spawn>
{
public:
  TurtleSpawn( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<turtlesim::Spawn>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<float>("position_x"),
      InputPort<float>("position_y"),
      InputPort<float>("position_theta"),
      InputPort<std::string>("turtle_name"),
      OutputPort<std::string>("result_name") };  
  }

  void sendRequest(RequestType& request) override
  {
    getInput("position_x", request.x);
    getInput("position_y", request.y);
    getInput("position_theta", request.theta);
    getInput("turtle_name", request.name);
    expected_result_ = request.name;
    ROS_INFO("turtlesim: sending request");
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("turtlesim: response received");
    if( rep.name == expected_result_)
    {
      setOutput<std::string>("result_name", rep.name);  
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_ERROR("AddTwoInts replied something unexpected: %s", rep.name);
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("turtlesim request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

private:
  std::string expected_result_;
};


#endif