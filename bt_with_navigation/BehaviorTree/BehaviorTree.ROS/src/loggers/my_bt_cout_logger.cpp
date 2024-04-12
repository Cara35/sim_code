#include "behaviortree_ros/loggers/my_bt_cout_logger.h"

#include<behaviortree_ros/Logger.h>

namespace BT
{
std::atomic<bool> MyStdCoutLogger::ref_count(false);

MyStdCoutLogger::MyStdCoutLogger(const BT::Tree& tree) : StatusChangeLogger(tree.rootNode())
{
  bool expected = false;
  // 如果expected==ref_count，就令ref_count=true（第二个参数），并返回true；
  // 否则，将ref_count的值赋给expected，并返回false
  // 即，ref_count初始化为false。第1次执行构造函数时，ref_count会被置为true，并返回true
  // 后面再进入构造函数时，expected!=ref_count，会抛出异常。
  if (!ref_count.compare_exchange_strong(expected, true))
  {
    throw LogicError("Only one instance of MyStdCoutLogger shall be created");
  }
  pub_log = nh.advertise<behaviortree_ros::Logger>("bt_logger",10);  //发布话题
}
MyStdCoutLogger::~MyStdCoutLogger()
{
  ref_count.store(false);
}

void MyStdCoutLogger::callback(Duration timestamp, const TreeNode& node,
                             NodeStatus prev_status, NodeStatus status)
{
  using namespace std::chrono;
  std::string node_ =node.name().c_str();
  std::string previous_status = toStr(prev_status).c_str();
  std::string current_status  = toStr(status).c_str();


  constexpr const char* whitespaces = "                         ";
  constexpr const size_t ws_count = 25;

  double since_epoch = duration<double>(timestamp).count();

  behaviortree_ros::Logger logger;
  logger.node = node_;
  logger.previous_status = previous_status;
  logger.current_status = current_status;

  pub_log.publish(logger);

  printf("[%.3f]: %s%s %s -> %s", since_epoch, node.name().c_str(),
         &whitespaces[std::min(ws_count, node.name().size())],
         toStr(prev_status, true).c_str(), toStr(status, true).c_str());
  std::cout << std::endl;
  // 当状态改变为SUCCESS，则认为它执行成功一次行为
  if(status == NodeStatus::SUCCESS){
      execute_count++;
  }
}

void MyStdCoutLogger::flush()
{
  std::cout << std::flush;
  ref_count = false;
}


}   // namespace BT
