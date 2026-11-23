#ifndef GRAPHEN_BT_NODES__PLAYBACK_NODES_HPP_
#define GRAPHEN_BT_NODES__PLAYBACK_NODES_HPP_

#include <string>

#include "graphen_bt_nodes/playback_action_node.hpp"
#include "graphen_interfaces/action/play_bvh.hpp"
#include "graphen_interfaces/action/play_choreo.hpp"
#include "graphen_interfaces/action/run_policy.hpp"
#include "graphen_interfaces/action/switch_mode.hpp"

namespace graphen::bt {

// 要求 SwitchGroupController 播放某個東西的四個節點，一種類型一個。
// 共通的行為與 port（action_name、timeout_ms、wait_finished、on_same、finish_timeout_ms）
// 見 PlaybackActionNode；這裡只列出各自的 port。

// 切換到系統模式（controllers.yaml 的 modes：target / hold / plugin），例如停止。
//   mode (string) 模式名稱
class SwitchMode : public PlaybackActionNode<graphen_interfaces::action::SwitchMode> {
public:
  SwitchMode(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();

protected:
  std::string fill_goal(Goal & goal) override;
};

// 播放編舞：舞曲庫中已發佈的名稱，或直接帶腳本內容（兩者擇一）。
// 表演用名稱；content 給由樹即時產生的短編舞（例如 JointVelocity 的速度命令）使用，
// 播放前一樣經過 controller 的完整檢查。
//   choreo  (string) 舞曲的名稱
//   content (string) 腳本的 YAML 內容
class PlayChoreo : public PlaybackActionNode<graphen_interfaces::action::PlayChoreo> {
public:
  PlayChoreo(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();

protected:
  std::string fill_goal(Goal & goal) override;
};

// 播放 BVH 目錄中的檔案。
//   bvh        (string) 檔名（不含 .bvh）
//   loop       (bool)   預設 true。播放到結尾後是否循環；循環播放不會結束（wait_finished 不會回傳）
//   time_scale (double) 預設 1.0。播放速度倍率，必須 > 0
class PlayBvh : public PlaybackActionNode<graphen_interfaces::action::PlayBvh> {
public:
  PlayBvh(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();

protected:
  std::string fill_goal(Goal & goal) override;
};

// 執行 policy 庫中的 policy。第一次執行需要載入（可能數百毫秒），timeout_ms 要涵蓋這段時間，
// 或事先以 controller 的 preload service 載入。
//   policy (string) policy 的名稱
class RunPolicy : public PlaybackActionNode<graphen_interfaces::action::RunPolicy> {
public:
  RunPolicy(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();

protected:
  std::string fill_goal(Goal & goal) override;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__PLAYBACK_NODES_HPP_
