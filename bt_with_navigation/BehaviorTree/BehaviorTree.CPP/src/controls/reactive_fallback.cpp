/* Copyright (C) 2020 Davide Faconti, Eurecat -  All Rights Reserved
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

#include "behaviortree_cpp_v3/controls/reactive_fallback.h"

namespace BT
{
NodeStatus ReactiveFallback::tick()
{
  // 记录失败的子节点数量
  size_t failure_count = 0;

  // 遍历所有子节点
  for (size_t index = 0; index < childrenCount(); index++)
  {
    // 获取当前子节点
    TreeNode* current_child_node = children_nodes_[index];
    
    // 执行当前子节点，获取其执行状态
    const NodeStatus child_status = current_child_node->executeTick();

    // 根据子节点的执行状态进行处理
    switch (child_status)
    {
      case NodeStatus::RUNNING: {
        // 如果当前子节点正在运行，将之前的子节点置为 IDLE 状态，
        // 确保它们在下次 tick 时重新进入运行状态
        for (size_t i = 0; i < index; i++)
        {
          haltChild(i);
        }
        // 返回 RUNNING，表示整个 ReactiveFallback 节点也处于运行状态
        return NodeStatus::RUNNING;
      }

      case NodeStatus::FAILURE: {
        // 如果当前子节点失败，增加失败计数
        failure_count++;
      }
      break;

      case NodeStatus::SUCCESS: {
        // 如果当前子节点成功，重置所有子节点状态，并返回 SUCCESS
        resetChildren();
        return NodeStatus::SUCCESS;
      }

      case NodeStatus::IDLE: {
        // 如果子节点返回了 IDLE，这是一个逻辑错误，抛出异常
        throw LogicError("A child node must never return IDLE");
      }
    }   // end switch
  }     //end for

  // 如果所有子节点都失败，重置所有子节点状态，并返回 FAILURE
  if (failure_count == childrenCount())
  {
    resetChildren();
    return NodeStatus::FAILURE;
  }

  // 如果执行到这里，表示当前 ReactiveFallback 节点的所有子节点都处于 RUNNING 或 FAILURE 状态，
  // 返回 RUNNING，表示整个 ReactiveFallback 节点也处于运行状态
  return NodeStatus::RUNNING;
}


}   // namespace BT
