#ifndef GRAPHEN_CONTROL_UTILS__TRACKING_PUBLISHER_HPP_
#define GRAPHEN_CONTROL_UTILS__TRACKING_PUBLISHER_HPP_

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "graphen_control_utils/state_interface_publisher.hpp"

namespace graphen::control {

// 將一組通道的「預期 / 實際 / 誤差」以 control_msgs/DynamicJointState 依指定頻率發布。
// 每個通道在 msg 中是一組 joint_names（名稱即建構時傳入的通道名稱，例如 L_KNEE_P/position），
// interface_names 為 expected、actual、error；error = expected − actual，任一方為 NaN 時為 NaN。
//
// 用法：RT 迴圈中每個通道呼叫 set()，最後呼叫 publish()。
// 值的緩衝在建構時配置，set() / publish() 不配置記憶體。
class TrackingPublisher {
public:
  TrackingPublisher(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & topic,
    const std::vector<std::string> & channel_names,
    double publish_rate)
  : values_(channel_names.size() * kValuesPerChannel, std::numeric_limits<double>::quiet_NaN()),
    publisher_(node, topic, make_interface_names(channel_names), publish_rate) {}

  size_t size() const {return values_.size() / kValuesPerChannel;}

  // 設定第 i 個通道的預期值與實際值；超出範圍時忽略
  void set(size_t i, double expected, double actual) {
    if (i >= size()) {
      return;
    }
    const size_t base = i * kValuesPerChannel;
    values_[base] = expected;
    values_[base + 1] = actual;
    // NaN 參與運算的結果本來就是 NaN；inf 相減也可能得到 NaN，統一視為無效
    const double error = expected - actual;
    values_[base + 2] = std::isfinite(error) ? error : std::numeric_limits<double>::quiet_NaN();
  }

  void publish(const rclcpp::Time & time, const rclcpp::Duration & period) {
    publisher_.publish(values_, time, period);
  }

private:
  static constexpr size_t kValuesPerChannel = 3;

  static std::vector<std::string> make_interface_names(const std::vector<std::string> & channels) {
    std::vector<std::string> names;
    names.reserve(channels.size() * kValuesPerChannel);
    for (const auto & channel : channels) {
      for (const char * suffix : {"/expected", "/actual", "/error"}) {
        names.push_back(channel + suffix);
      }
    }
    return names;
  }

  std::vector<double> values_;  // [通道 0 的 expected, actual, error, 通道 1 的 ...]
  StateInterfacePublisher publisher_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROL_UTILS__TRACKING_PUBLISHER_HPP_
