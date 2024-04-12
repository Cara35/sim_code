#include <ros/ros.h>
#include <iostream>
#include <ros/package.h>
#include "behaviortree_ros/loggers/my_bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h" //log
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "example_behaviortree/MoveBasePoint.h"
#include "example_behaviortree/take_photo.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_behavior_tree");
  ros::NodeHandle nh;
  std::string xml_path = ros::package::getPath("behaviortree_ros") + "/config/" ;
  std::string file_name = "bt_action_point.xml";
  // 创建行为树工厂
  BehaviorTreeFactory factory;
  
  // 注册行为树节点类型
  RegisterRosAction<MoveBasePoint>(factory, "MoveBaseGoal", nh);
  RegisterRosService<TakePhoto>(factory, "TakePhoto", nh);

  // 从XML文本创建行为树
  auto tree = factory.createTreeFromFile(xml_path+file_name);

  double total_action = tree.nodes.size();
  std::cout<<"总行为数量："<<total_action<<std::endl;

  MyStdCoutLogger logger_cout(tree);  //在终端显示log信息
  FileLogger logger_file(tree,"bt_trace.fbl");  //在文件记录log信息，可在Groot上回放
  MinitraceLogger logger_minitrace(tree,"bt_trace.json"); //用于保存节点的执行时序
  PublisherZMQ publisher_zmq(tree);   //在节点执行时同时发布其状态变化，在Groot中实时观察

  // printTreeRecursively(tree.rootNode());  //层级打印树结构，默认打印在终端

  NodeStatus status = NodeStatus::IDLE;
  //只有当root节点的NodeStatus为IDLE和RUNNING时才可执行tick()函数
  while(ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    ros::spinOnce();
    // status经过tickRoot后就是root节点的状态
    status = tree.tickRoot();
    double current_execute_number = logger_cout.getExecuteCount();
    // std::cout<<"执行的次数为：------------- "<<current_execute_number<<std::endl;
    // std::cout<<"任务总进度为：------------- "<<( current_execute_number / total_action ) * 100 <<"% ---------"<<std::endl;

    //在终端只会输出 statue:RUNNING，因为在循环里中，status经过status = tree.tickRoot()后它的状态就一直是RUNNING状态，在tickRoot前它是IDLE状态。
    // std::cout <<"status:  " <<status << std::endl;  
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }
    

  return 0;
}


