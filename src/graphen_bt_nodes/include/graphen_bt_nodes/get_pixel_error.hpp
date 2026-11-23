#ifndef GRAPHEN_BT_NODES__GET_PIXEL_ERROR_HPP_
#define GRAPHEN_BT_NODES__GET_PIXEL_ERROR_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace graphen::bt {

// 外部視覺節點的目標像素（例如人臉中心）→ 以畫面中心為原點的正規化誤差。
//
// 訂閱 geometry_msgs/PointStamped（point.x / point.y 為像素，原點在左上角）。新舊以「收到的時間」判斷
// （steady clock），不使用 header.stamp：發送端的時鐘來源可能和這裡不同。
// QoS 為 SensorData（best effort），可以收 reliable 與 best effort 的發送端。
//
// Ports：
//   topic        (string) 只在建構時讀取
//   image_width  (double) 影像寬度（px）
//   image_height (double) 影像高度（px）
//   timeout_ms   (int)    預設 500。超過這段時間沒有新資料視為目標遺失
//   err_x/err_y  (double, output) 正規化誤差 (-1..1)：右 / 下為正
//   abs_x/abs_y  (double, output) |err|，讓樹可以直接用 IsAbove 比較門檻
//   dir_x/dir_y  (double, output) err 的正負號（-1 / 0 / 1）
//   lost_s       (double, output) 距離上次收到資料的秒數；從未收到時為很大的值。遺失時也會輸出
//
// 有新鮮資料時回傳 SUCCESS，遺失時回傳 FAILURE。
class GetPixelError : public BT::SyncActionNode {
public:
  GetPixelError(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  struct Latest {
    std::mutex mutex;
    std::optional<geometry_msgs::msg::Point> point;
    std::chrono::steady_clock::time_point received;
  };

  std::shared_ptr<Latest> latest_{std::make_shared<Latest>()};
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};

}  // namespace graphen::bt

#endif  // GRAPHEN_BT_NODES__GET_PIXEL_ERROR_HPP_
