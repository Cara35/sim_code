#ifndef MY_BT_COUT_LOGGER_H
#define MY_BT_COUT_LOGGER_H

#include <cstring>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include<ros/ros.h>
namespace BT
{
/**
 * @brief AddMyStdCoutLoggerToTree. Give  the root node of a tree,
 * a simple callback is subscribed to any status change of each node.
 *
 *
 * @param root_node
 * @return Important: the returned shared_ptr must not go out of scope,
 *         otherwise the logger is removed.
 */

class MyStdCoutLogger : public StatusChangeLogger
{
  static std::atomic<bool> ref_count;

public:
  ros::NodeHandle nh;
  ros::Publisher pub_log;

  MyStdCoutLogger(const BT::Tree& tree);
  ~MyStdCoutLogger() override;
  //当状态改变-->触发回调,在终端打印状态信息
  virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                        NodeStatus status) override;
  double getExecuteCount() {
    return execute_count;
  }
  virtual void flush() override;
  
private:
  double execute_count;
};

}   // namespace BT


#endif   // BT_COUT_LOGGER_H

