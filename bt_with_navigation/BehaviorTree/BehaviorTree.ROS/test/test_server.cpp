/**
 * @file test_server.cpp
 * @brief 此代码示例展示了如何使用ROS和行为树来创建一个ROS服务（add_two_ints）和一个ROS行为（fibonacci）。
 * Add函数处理add_two_ints服务请求，而FibonacciServer类处理fibonacci行为请求。
 * 行为fibonacci会生成一个Fibonacci数列，直到达到指定的顺序，同时还支持抢占操作。
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behaviortree_ros/AddTwoInts.h>
#include <behaviortree_ros/FibonacciAction.h>

// 用于处理服务请求的回调函数
bool Add(behaviortree_ros::AddTwoInts::Request  &req,
         behaviortree_ros::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%d, y=%d", req.a, req.b);
  ROS_INFO("sending back response: [%d]", res.sum);
  return true;
}

// FibonacciServer类定义，用于处理FibonacciAction
class FibonacciServer
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behaviortree_ros::FibonacciAction> server_; // 创建一个SimpleActionServer
  std::string action_name_;
  behaviortree_ros::FibonacciFeedback feedback_; // Fibonacci反馈消息
  behaviortree_ros::FibonacciResult result_; // Fibonacci结果消息
  int call_number_;

public:
  FibonacciServer(std::string name) :
    server_(nh_, name, boost::bind(&FibonacciServer::executeCB, this, _1), false),
    action_name_(name)
  {
    server_.start(); // 启动SimpleActionServer
    call_number_ = 0;
  }

  ~FibonacciServer(void)
  {
  }

  void executeCB(const behaviortree_ros::FibonacciGoalConstPtr &goal)
  {
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i",
             action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    for(int i=1; i<=goal->order; i++)
    {
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      server_.publishFeedback(feedback_); // 发布Fibonacci反馈消息
    }

    bool preempted = false;
    int required_time = 500;

    if( call_number_ % 2 == 0)
    {
      required_time = 100;
    }

    while ( required_time > 0 )
    {
      if (server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        server_.setPreempted();
        preempted = true;
        break;
      }
      ros::Duration take_break(0.010);
      take_break.sleep();
      required_time -= 10;
    }

    if(!preempted)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      server_.setSucceeded(result_);
    }
    else{
      ROS_WARN("%s: Preempted", action_name_.c_str());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server"); // 初始化ROS节点
  ros::NodeHandle n;

  // 创建一个ROS服务，用于处理AddTwoInts请求
  ros::ServiceServer service = n.advertiseService("add_two_ints", Add);
  ROS_INFO("Ready to add two ints.");

  // 创建FibonacciServer实例，用于处理FibonacciAction请求
  FibonacciServer fibonacci("fibonacci");
  ros::spin(); // 进入ROS事件循环

  return 0;
}
