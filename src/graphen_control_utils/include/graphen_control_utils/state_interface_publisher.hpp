#ifndef GRAPHEN_CONTROL_UTILS__STATE_INTERFACE_PUBLISHER_HPP_
#define GRAPHEN_CONTROL_UTILS__STATE_INTERFACE_PUBLISHER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace graphen::control {

// 將任意一組具名 double 值（state interface、reference 等）以
// control_msgs/DynamicJointState 依指定頻率發布。
// 名稱以最後一個 '/' 拆成 prefix 與 interface（例如 joint_arm1/position），
// 依 prefix 分組；publish() 傳入值的順序需與建構時的 interface_names 相同。
class StateInterfacePublisher {
public:
  using MsgT = control_msgs::msg::DynamicJointState;

  StateInterfacePublisher(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & topic,
    const std::vector<std::string> & interface_names,
    double publish_rate)
  : publish_period_(publish_rate > 0.0 ? 1.0 / publish_rate : 0.0) {
    publisher_ = node->create_publisher<MsgT>(topic, rclcpp::SystemDefaultsQoS());
    rt_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<MsgT>>(publisher_);

    // 預先配置 msg，update() 內只填值不配置記憶體
    auto & msg = rt_publisher_->msg_;
    index_map_.reserve(interface_names.size());
    for (const auto & full_name : interface_names) {
      const auto pos = full_name.rfind('/');
      const std::string prefix = (pos == std::string::npos) ? std::string{} : full_name.substr(0, pos);
      const std::string iface = (pos == std::string::npos) ? full_name : full_name.substr(pos + 1);

      size_t group = 0;
      while (group < msg.joint_names.size() && msg.joint_names[group] != prefix) {
        ++group;
      }
      if (group == msg.joint_names.size()) {
        msg.joint_names.push_back(prefix);
        msg.interface_values.emplace_back();
      }

      auto & values = msg.interface_values[group];
      values.interface_names.push_back(iface);
      values.values.push_back(std::numeric_limits<double>::quiet_NaN());
      index_map_.emplace_back(group, values.values.size() - 1);
    }
  }

  // 發布 state interface 的目前值（順序需與 interface_names 相同）
  void publish(
    const std::vector<hardware_interface::LoanedStateInterface> & states,
    const rclcpp::Time & time, const rclcpp::Duration & period) {
    publish_impl(
      states.size(), [&states](size_t i) { return states[i].get_value(); }, time, period);
  }

  // 發布一般數值陣列（例如 chainable controller 的 reference_interfaces_）
  void publish(
    const std::vector<double> & values,
    const rclcpp::Time & time, const rclcpp::Duration & period) {
    publish_impl(values.size(), [&values](size_t i) { return values[i]; }, time, period);
  }

private:
  // 共用的節流與填值邏輯；get_value(i) 回傳第 i 個值
  template<typename GetValueT>
  void publish_impl(
    size_t size, GetValueT && get_value,
    const rclcpp::Time & time, const rclcpp::Duration & period) {
    if (size != index_map_.size()) {
      return;
    }

    elapsed_ += period.seconds();
    if (elapsed_ < publish_period_) {
      return;
    }

    if (rt_publisher_->trylock()) {
      // 保留餘數以維持準確頻率；落後太多時（例如卡住）直接歸零，避免連發
      elapsed_ -= publish_period_;
      if (elapsed_ >= publish_period_) {
        elapsed_ = 0.0;
      }
      auto & msg = rt_publisher_->msg_;
      msg.header.stamp = time;
      for (size_t i = 0; i < size; ++i) {
        const auto & [group, slot] = index_map_[i];
        msg.interface_values[group].values[slot] = get_value(i);
      }
      rt_publisher_->unlockAndPublish();
    }
  }

  double publish_period_;
  double elapsed_{0.0};
  std::vector<std::pair<size_t, size_t>> index_map_;  // 輸入值 index -> (group, slot)

  rclcpp::Publisher<MsgT>::SharedPtr publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<MsgT>> rt_publisher_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROL_UTILS__STATE_INTERFACE_PUBLISHER_HPP_
