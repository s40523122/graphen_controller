#ifndef GRAPHEN_BT_NODES__JOINT_VELOCITY_HPP_
#define GRAPHEN_BT_NODES__JOINT_VELOCITY_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "graphen_bt_nodes/playback_action_node.hpp"
#include "graphen_interfaces/action/play_choreo.hpp"
#include "rclcpp/rclcpp.hpp"

namespace graphen::bt {

// 以「速度」指揮關節：把各 joint 的速度換成一段 linear 編舞，以 PlayChoreo 的 content 送出。
//
// 每個 joint 一條軌道：目標 = 目前位置 + v·horizon（截在 [lower, upper] 內），時間依距離 / |v|，
// 所以是等速移動；v = 0 或已在邊界的 joint 寫成 hold（停在切換當下的命令位置）。
// 命令只維持 horizon 秒（有時效）：要持續移動，樹必須在到期前再送一次（SendOnChange 的 refresh_ms），
// BT 中斷或續送失敗時，頭部最多再移動 v·horizon 就停下。
// 續送時新的編舞從切換當下的「命令位置」開始、以相同速度移動，所以位置與速度都是連續的。
//
// 速度、限位、max_steps 不在這裡限制：超過 controller 上限（choreo.velocity_limit_scale、
// choreo.max_steps、URDF 限位）的命令會被 controller 拒絕（FAILURE），不會被默默降速。
// 目前位置讀取量測值（~/states 的 position）；編舞的起點是命令位置，兩者的差距（追蹤落後）
// 會讓實際速度略低於指定值。
//
// Ports（共通的 port 見 PlaybackActionNode；on_same 預設 restart，續送時一律重新開始計時）：
//   joints       (string)         joint 名稱，以 ; 分隔
//   velocities   (string)         各 joint 的速度（deg/s），以 ; 分隔，數量同 joints
//   lower/upper  (string)         各 joint 的移動範圍（rad），以 ; 分隔，可以比 URDF 限位小
//   horizon      (double)         命令的有效時間（s），預設 1.0
//   states_topic (string)         只在建構時讀取，預設 /switch_group_controller/states
//
// 還沒收到 joint 狀態時回傳 FAILURE；port 缺少或不合法時丟出 BT::RuntimeError。
class JointVelocity : public PlaybackActionNode<graphen_interfaces::action::PlayChoreo> {
public:
  JointVelocity(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

protected:
  std::string fill_goal(Goal & goal) override;

private:
  struct Latest {
    std::mutex mutex;
    control_msgs::msg::DynamicJointState::ConstSharedPtr msg;
  };

  // msg 中各 joint 的 position；缺少任何一個時回傳 false
  static bool read_positions(
    const control_msgs::msg::DynamicJointState & msg, const std::vector<std::string> & joints,
    std::vector<double> & positions);

  rclcpp::Logger node_logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::shared_ptr<Latest> latest_{std::make_shared<Latest>()};
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr subscription_;
  // onStart 讀到的目前位置，給 fill_goal 使用
  std::vector<std::string> joints_;
  std::vector<double> positions_;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__JOINT_VELOCITY_HPP_
