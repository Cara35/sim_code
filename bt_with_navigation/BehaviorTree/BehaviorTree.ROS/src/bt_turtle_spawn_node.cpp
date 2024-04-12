#include <ros/ros.h>
#include <iostream>
#include <ros/package.h>
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h" //在终端上显示任务进程

#include "example_behaviortree/TurtleSpawn.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_behavior_tree");
  ros::NodeHandle nh;
  std::string xml_path = ros::package::getPath("behaviortree_ros") + "/config/" ;
  std::string file_name = "bt_turtle_spawn.xml";

  // 创建行为树工厂
  BehaviorTreeFactory factory;

  // 注册行为树节点类型
  RegisterRosService<TurtleSpawn>(factory, "Fallturtle", nh);

  // 从XML文本创建行为树
  auto tree = factory.createTreeFromFile(xml_path+file_name);

  StdCoutLogger logger_cout(tree);
  
  NodeStatus status = NodeStatus::IDLE;
  status = tree.tickRoot();
  std::cout <<"status:  "  << status<<std::endl;

  return 0;
}
