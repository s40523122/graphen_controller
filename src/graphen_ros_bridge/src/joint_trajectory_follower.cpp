#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "graphen_locomotion/locomotion_algorithm.hpp"
#include "graphen_ros_bridge/trajectory_sampler.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace graphen::ros_bridge {

using locomotion::Context;
using locomotion::JointCommand;
using locomotion::LogLevel;
using locomotion::RobotState;

// 訂閱 trajectory_msgs/JointTrajectory（例如 GR00T / VLA 的關節命令），在 planner thread 中
// 依 time_from_start 取樣，寫入 JointCommand：
//   positions → q*，velocities → q̇*，kp / kd → 該關節的 PD 增益（有設定時）
// 下游 PdController（hardware:=mujoco_pd / real_pd）以 τ = kp·(q* − q) + kd·(q̇* − q̇) 計算力矩；
// 沒有設定 kp / kd 時保持 NaN，PdController 用自己的預設增益。PositionController 會忽略增益。
//
// 這個 plugin 需要自己建立 rclcpp node（輸入來自 ROS topic），所以不放在 graphen_locomotion
// （該套件的介面與內建 plugin 不依賴 rclcpp，見 ARCHITECTURE D49）。取樣邏輯在
// trajectory_sampler.hpp，與傳輸方式無關。
//
// 參數（<mode>.*）：
//   topic            訂閱的 topic（必填）
//   qos_reliability  best_effort（預設）/ reliable。要和發布端相容：BEST_EFFORT 的發布端
//                    不會送給 RELIABLE 的訂閱
//   timeout          s（預設 0.5，>= 0）。軌跡播完後超過這段時間沒有新的訊息，速度前饋改為 0
//                    並 WARN 一次（位置停在最後一點）。發布端以單點連續送出時，最後一點的速度
//                    只在下一筆到達前有效，發布端停止後不能一直保留
//   log_period       s（預設 1.0，> 0）。拒絕訊息的 ERROR 至少間隔這麼久才再印一次
//   kp、kd           （選填）一個數字套到全部關節，或依 Context::joint_names 順序各給一個值（>= 0），
//                    必須同時給或同時省略
//
// 只播放這個模式進入之後收到的訊息：進入模式時丟掉之前的軌跡。header.stamp 不使用，
// 時間原點是 planner 第一次看到這筆軌跡的週期，之後以 update() 的 dt 累加（和 planner、sim time 一致）。
// 只有一個點時立刻採用：發布端約以 60 Hz 送出單點、time_from_start 等於週期，
// 若等到那個時間才輸出，下一筆就先把時鐘歸零，命令永遠不會生效。
// 訊息裡沒有的關節不寫命令，維持目前位置與下游預設增益。
class JointTrajectoryFollower : public locomotion::LocomotionAlgorithm {
public:
  JointTrajectoryFollower() = default;
  ~JointTrajectoryFollower() override {stop_spin();}

  JointTrajectoryFollower(const JointTrajectoryFollower &) = delete;
  JointTrajectoryFollower & operator=(const JointTrajectoryFollower &) = delete;

  void configure(const Context & context) override {
    const auto topic = context.params.get_string("topic");
    if (!topic || topic->empty()) {
      throw std::invalid_argument("'topic' must be set");
    }
    topic_ = *topic;
    const auto reliability = context.params.get_string("qos_reliability").value_or("best_effort");
    if (reliability != "best_effort" && reliability != "reliable") {
      throw std::invalid_argument("'qos_reliability' must be best_effort or reliable");
    }
    timeout_ = context.params.get_double("timeout").value_or(0.5);
    if (!std::isfinite(timeout_) || timeout_ < 0.0) {
      throw std::invalid_argument("'timeout' must be finite and >= 0");
    }
    const double log_period = context.params.get_double("log_period").value_or(1.0);
    if (!std::isfinite(log_period) || !(log_period > 0.0)) {
      throw std::invalid_argument("'log_period' must be greater than zero");
    }
    log_period_ = std::chrono::duration<double>(log_period);
    joint_names_ = context.joint_names;
    log_ = context.log;
    kp_ = read_gains(context, "kp");
    kd_ = read_gains(context, "kd");
    if (kp_.empty() != kd_.empty()) {
      throw std::invalid_argument("'kp' and 'kd' must both be set or both be omitted");
    }

    // 不套用程序的全域參數：ros2_control_node 的 __node 等 remap 會讓所有 node 同名
    static std::atomic<int> next_id{0};
    const auto name = "joint_trajectory_follower_" + std::to_string(next_id.fetch_add(1));
    node_ = std::make_shared<rclcpp::Node>(
      name, rclcpp::NodeOptions()
      .use_global_arguments(false)
      .start_parameter_services(false)
      .start_parameter_event_publisher(false));
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    if (reliability == "best_effort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }
    subscription_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      topic_, qos,
      [this](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {accept(*msg);});
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] {executor_->spin();});

    log(LogLevel::kInfo,
      "subscribing JointTrajectory on " + topic_ + " (" + reliability + "); " +
      (kp_.empty() ? "PD uses the downstream controller default gains" :
      "PD gains come from kp/kd"));
  }

  void on_enter(const RobotState & /*state*/) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.reset();
    }
    current_.reset();
    elapsed_ = 0.0;
    stale_warned_ = false;
    log(LogLevel::kInfo, "waiting for a JointTrajectory on " + topic_);
  }

  void update(double dt, const RobotState & /*state*/, JointCommand & command) override {
    std::shared_ptr<const Trajectory> latest;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest = std::move(pending_);
      pending_.reset();
    }
    if (latest) {
      // 新的軌跡從這個週期開始播放（t = 0）
      current_ = std::move(latest);
      elapsed_ = 0.0;
      stale_warned_ = false;
    } else if (current_) {
      elapsed_ += dt;
    }
    if (!current_) {
      return;
    }

    const bool past_end = sample(*current_, elapsed_, command);
    if (past_end && elapsed_ - current_->end_time() > timeout_) {
      for (const int index : current_->command_index) {
        if (index >= 0) {
          command.velocities[static_cast<size_t>(index)] = 0.0;
        }
      }
      if (!stale_warned_) {
        stale_warned_ = true;
        log(LogLevel::kWarn,
          "no new JointTrajectory on " + topic_ + " after the last point; holding position");
      }
    }
    if (!kp_.empty() && elapsed_ >= current_->points.front().time) {
      for (const int index : current_->command_index) {
        if (index >= 0) {
          const auto i = static_cast<size_t>(index);
          command.kp[i] = kp_[i];
          command.kd[i] = kd_[i];
        }
      }
    }
  }

private:
  void stop_spin() {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    subscription_.reset();
    executor_.reset();
    node_.reset();
  }

  void log(LogLevel level, const std::string & message) const {
    if (log_) {
      log_(level, message);
    }
  }

  // 一個值套到每個關節；多個值必須對上 joint_names。未設定時回傳空 vector
  static std::vector<double> read_gains(const Context & context, const char * name) {
    std::optional<std::vector<double>> values;
    try {
      values = context.params.get_double_array(name);
    } catch (const std::invalid_argument &) {
      // 不是陣列：接受單一數字，其他型別照樣丟出
      if (const auto value = context.params.get_double(name)) {
        values = std::vector<double>{*value};
      } else {
        throw;
      }
    }
    if (!values) {
      return {};
    }
    if (values->size() == 1) {
      values->assign(context.joint_names.size(), values->front());
    } else if (values->size() != context.joint_names.size()) {
      throw std::invalid_argument(
              std::string("'") + name + "' must be one value or one value per commanded joint");
    }
    for (const double gain : *values) {
      if (!std::isfinite(gain) || gain < 0.0) {
        throw std::invalid_argument(std::string("'") + name + "' values must be finite and >= 0");
      }
    }
    return *values;
  }

  static double to_seconds(const builtin_interfaces::msg::Duration & duration) {
    return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
  }

  // 拒絕訊息的 log 以 log_period 節流（訂閱 thread 中呼叫）
  void reject(const std::string & reason) {
    const auto now = std::chrono::steady_clock::now();
    ++rejected_since_log_;
    if (last_reject_log_ && now - *last_reject_log_ < log_period_) {
      return;
    }
    log(LogLevel::kError,
      "rejected JointTrajectory on " + topic_ + ": " + reason +
      (rejected_since_log_ > 1 ? " (" + std::to_string(rejected_since_log_) + " rejected)" : ""));
    last_reject_log_ = now;
    rejected_since_log_ = 0;
  }

  void accept(const trajectory_msgs::msg::JointTrajectory & message) {
    auto trajectory = std::make_shared<Trajectory>();
    trajectory->command_index.assign(message.joint_names.size(), -1);
    size_t matched = 0;
    for (size_t joint = 0; joint < message.joint_names.size(); ++joint) {
      const auto it = std::find(joint_names_.begin(), joint_names_.end(), message.joint_names[joint]);
      if (it != joint_names_.end()) {
        trajectory->command_index[joint] = static_cast<int>(it - joint_names_.begin());
        ++matched;
      }
    }
    trajectory->points.reserve(message.points.size());
    for (const auto & source : message.points) {
      TrajectoryPoint point;
      point.time = to_seconds(source.time_from_start);
      point.positions = source.positions;
      point.velocities = source.velocities;
      trajectory->points.push_back(std::move(point));
    }
    if (const auto error = validate(*trajectory)) {
      reject(*error);
      return;
    }
    if (trajectory->points.size() == 1) {
      trajectory->points.front().time = 0.0;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_ = std::move(trajectory);
    }
    if (!logged_accept_) {
      logged_accept_ = true;
      log(LogLevel::kInfo,
        "accepted JointTrajectory on " + topic_ + " (" + std::to_string(message.points.size()) +
        " points, " + std::to_string(matched) + " joints, ignored " +
        std::to_string(message.joint_names.size() - matched) + ")");
    }
  }

  // configure 後固定
  std::string topic_;
  std::vector<std::string> joint_names_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  double timeout_{0.5};
  std::chrono::duration<double> log_period_{1.0};
  locomotion::Logger log_;

  // 訂閱 thread
  bool logged_accept_{false};
  std::optional<std::chrono::steady_clock::time_point> last_reject_log_;
  size_t rejected_since_log_{0};

  // planner thread
  std::shared_ptr<const Trajectory> current_;
  double elapsed_{0.0};
  bool stale_warned_{false};

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  // 訂閱 thread 寫入、planner thread 取走
  std::mutex mutex_;
  std::shared_ptr<const Trajectory> pending_;
};

}  // namespace graphen::ros_bridge

PLUGINLIB_EXPORT_CLASS(
  graphen::ros_bridge::JointTrajectoryFollower,
  graphen::locomotion::LocomotionAlgorithm)
