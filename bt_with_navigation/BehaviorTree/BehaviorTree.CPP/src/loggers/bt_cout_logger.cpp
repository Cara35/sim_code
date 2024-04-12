#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

namespace BT
{
std::atomic<bool> StdCoutLogger::ref_count(false);

StdCoutLogger::StdCoutLogger(const BT::Tree& tree) : StatusChangeLogger(tree.rootNode())
{
  bool expected = false;
  // 如果expected==ref_count，就令ref_count=true（第二个参数），并返回true；
  // 否则，将ref_count的值赋给expected，并返回false
  // 即，ref_count初始化为false。第1次执行构造函数时，ref_count会被置为true，并返回true
  // 后面再进入构造函数时，expected!=ref_count，会抛出异常。
  if (!ref_count.compare_exchange_strong(expected, true))
  {
    throw LogicError("Only one instance of StdCoutLogger shall be created");
  }
}
StdCoutLogger::~StdCoutLogger()
{
  ref_count.store(false);
}

void StdCoutLogger::callback(Duration timestamp, const TreeNode& node,
                             NodeStatus prev_status, NodeStatus status)
{
  using namespace std::chrono;

  constexpr const char* whitespaces = "                         ";
  constexpr const size_t ws_count = 25;

  double since_epoch = duration<double>(timestamp).count();
  printf("[%.3f]: %s%s %s -> %s", since_epoch, node.name().c_str(),
         &whitespaces[std::min(ws_count, node.name().size())],
         toStr(prev_status, true).c_str(), toStr(status, true).c_str());
  std::cout << std::endl;
}

void StdCoutLogger::flush()
{
  std::cout << std::flush;
  ref_count = false;
}

}   // namespace BT
