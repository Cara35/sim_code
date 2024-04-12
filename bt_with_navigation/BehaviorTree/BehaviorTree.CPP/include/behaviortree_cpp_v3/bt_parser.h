/* Copyright (C) 2022 Davide Faconti -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"

namespace BT
{
/**
 * @brief BehaviorTreeParser是一个类，
 * 用于从文件或文本中读取行为树的模型，
 * 并使用BehaviorTreeFactory实例化相应的树。
 */
class Parser
{
public:
  Parser() = default;

  virtual ~Parser() = default;

  Parser(const Parser& other) = delete;
  Parser& operator=(const Parser& other) = delete;

  virtual void loadFromFile(const std::string& filename, bool add_includes = true) = 0;

  virtual void loadFromText(const std::string& xml_text, bool add_includes = true) = 0;

  virtual std::vector<std::string> registeredBehaviorTrees() const = 0;

  virtual Tree instantiateTree(const Blackboard::Ptr& root_blackboard,
                               std::string tree_name = {}) = 0;

  virtual void clearInternalState() {};
};

}   // namespace BT
