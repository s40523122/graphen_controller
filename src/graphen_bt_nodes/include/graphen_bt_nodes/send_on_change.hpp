#ifndef GRAPHEN_BT_NODES__SEND_ON_CHANGE_HPP_
#define GRAPHEN_BT_NODES__SEND_ON_CHANGE_HPP_

#include <chrono>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace graphen::bt {

// 低頻命令的節流：只在命令改變、或需要續送時才執行子節點（例如送出 action），其他 tick 直接 SUCCESS。
//
// 子節點在以下情況才會被 tick：
//   - value 和上一次子節點成功時不同
//   - 距離上一次成功已超過 refresh_ms（> 0 時；用來在有時效的命令到期前續送）
//   - 子節點仍在 RUNNING（送出中的命令要等它完成，期間 value 改變也不中斷）
// 子節點失敗時不更新記錄，下一個 tick 會再試一次。
// 被 halt 時清除記錄：重新進入這個分支時一定會送出一次。
//
// Ports：
//   value      (string) 命令的內容（比較用）
//   refresh_ms (int)    預設 0。> 0 時，命令不變也每隔這段時間重送一次；0 表示只在改變時送出
class SendOnChange : public BT::DecoratorNode {
public:
  SendOnChange(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  void halt() override;

private:
  BT::NodeStatus tick() override;

  std::optional<std::string> last_value_;
  std::chrono::steady_clock::time_point last_sent_;
  // 子節點 RUNNING 時，這次送出的 value（成功時記錄這個值，而不是完成當下的 value）
  std::string pending_value_;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__SEND_ON_CHANGE_HPP_
