#ifndef XML_PARSING_BT_H
#define XML_PARSING_BT_H

#include "behaviortree_cpp_v3/bt_parser.h"

#include <unordered_map>

namespace BT
{
/**
  * @brief XMLParser是一个用于读取模型的类
  *从文件或文本中实例化行为树
  *使用BehaviorTreeFactory生成相应的树。
  */
class XMLParser : public Parser
{
public:
  XMLParser(const BehaviorTreeFactory& factory);

  ~XMLParser() override;

  XMLParser(const XMLParser& other) = delete;
  XMLParser& operator=(const XMLParser& other) = delete;
  
  // 从指定文件加载XML文档，并在加载过程中处理包含include的相关逻辑
  void loadFromFile(const std::string& filename, bool add_includes = true) override;

  void loadFromText(const std::string& xml_text, bool add_includes = true) override;

  std::vector<std::string> registeredBehaviorTrees() const override;

  Tree instantiateTree(const Blackboard::Ptr& root_blackboard,
                       std::string main_tree_to_execute = {}) override;

  void clearInternalState() override;

private:
  struct Pimpl;
  Pimpl* _p;
};

// 函数作用：验证XML的有效性
void VerifyXML(const std::string& xml_text,
               const std::unordered_map<std::string, NodeType>& registered_nodes);

std::string writeTreeNodesModelXML(const BehaviorTreeFactory& factory,
                                   bool include_builtin = false);

}   // namespace BT

#endif   // XML_PARSING_BT_H
