#ifndef ABSTRACT_LOGGER_H
#define ABSTRACT_LOGGER_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace BT
{
enum class TimestampType
{
  absolute,
  relative
};

typedef std::array<uint8_t, 12> SerializedTransition;

//所有logger的基类，纯虚基类
class StatusChangeLogger
{
public:
  StatusChangeLogger(TreeNode* root_node);
  virtual ~StatusChangeLogger() = default;
  //当node发生状态变化时要执行的操作
  virtual void callback(BT::Duration timestamp, const TreeNode& node,
                        NodeStatus prev_status, NodeStatus status) = 0;
  //保存或发送数据
  virtual void flush() = 0;

  void setEnabled(bool enabled)
  {
    enabled_ = enabled;
  }

  void setTimestampType(TimestampType type)
  {
    type_ = type;
  }

  bool enabled() const
  {
    return enabled_;
  }

  // false by default.
  bool showsTransitionToIdle() const
  {
    return show_transition_to_idle_;
  }

  void enableTransitionToIdle(bool enable)
  {
    show_transition_to_idle_ = enable;
  }

private:
  bool enabled_;
  bool show_transition_to_idle_;
  std::vector<TreeNode::StatusChangeSubscriber> subscribers_;
  TimestampType type_;
  BT::TimePoint first_timestamp_;
};

//--------------------------------------------

inline StatusChangeLogger::StatusChangeLogger(TreeNode* root_node) :
  enabled_(true), show_transition_to_idle_(true), type_(TimestampType::absolute)
{
  first_timestamp_ = std::chrono::high_resolution_clock::now();
  //对回调函数callback()的封装，执行配置选项对应的callback()
  auto subscribeCallback = [this](TimePoint timestamp, const TreeNode& node,
                                  NodeStatus prev, NodeStatus status) {
    if (enabled_ && (status != NodeStatus::IDLE || show_transition_to_idle_))
    {
      if (type_ == TimestampType::absolute)
      { 
        // 真正的回调操作
        this->callback(timestamp.time_since_epoch(), node, prev, status);
      }
      else
      {
        this->callback(timestamp - first_timestamp_, node, prev, status);
      }
    }
  };
  // 增加订阅者，绑定回调函数
  auto visitor = [this, subscribeCallback](TreeNode* node) {
    subscribers_.push_back(node->subscribeToStatusChange(std::move(subscribeCallback)));
  };
  // 遍历树的所有节点
  applyRecursiveVisitor(root_node, visitor);
}
}   // namespace BT

#endif   // ABSTRACT_LOGGER_H
