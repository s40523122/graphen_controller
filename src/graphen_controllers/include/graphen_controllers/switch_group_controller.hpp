#ifndef GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_
#define GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "graphen_control_utils/state_interface_publisher.hpp"
#include "graphen_control_utils/try_lock_buffer.hpp"
#include "graphen_controllers/bvh_source.hpp"
#include "graphen_controllers/choreo_source.hpp"
#include "graphen_controllers/locomotion_planner.hpp"
#include "graphen_controllers/motion.hpp"
#include "graphen_controllers/playback_arbiter.hpp"
#include "graphen_controllers/playback_server.hpp"
#include "graphen_controllers/plugin_factory.hpp"
#include "graphen_controllers/policy_source.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "graphen_interfaces/action/play_bvh.hpp"
#include "graphen_interfaces/action/play_choreo.hpp"
#include "graphen_interfaces/action/run_policy.hpp"
#include "graphen_interfaces/action/switch_mode.hpp"
#include "graphen_interfaces/srv/get_choreo.hpp"
#include "graphen_interfaces/srv/list_choreos.hpp"
#include "graphen_interfaces/srv/preload.hpp"
#include "graphen_interfaces/srv/publish_choreo.hpp"
#include "graphen_locomotion/base_state_estimator.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/string.hpp"

namespace graphen::control {

// 頂層切換 controller：播放目前的動作，寫入下游子群組 controller（PositionController / PdController）
// 的 reference interface。
//
// 同一時間只播放一個 Motion，由 LocomotionPlanner 在 non-RT thread 中以 planner_rate 播放、
// 每週期最多移動 max_steps，寫入 target_buffer_；update() 只讀取最新值輸出。
// update() 另外把量測的關節角度 / 速度與 IMU 寫入 state_buffer_。planner 每週期讀取後，
// 補上 ~/cmd_vel 的速度指令、執行機身狀態估測器（base_estimator），再交給 Motion。
//
// 可以播放的東西依「模式」（類型）分成四種，各有自己的入口；controller 本身只做接線，
// 不包含任何特定模式的邏輯：
//   系統模式  controllers.yaml 的 modes 列出的具名模式（target / hold / plugin），啟動時就建立。
//             ~/switch_mode action 或 ~/mode topic。停止（hold）走這條路：不需要載入任何東西
//   choreo    編舞（ChoreoSource）：機器人端舞曲庫中已發佈的腳本，或直接帶內容試播。~/play_choreo
//   bvh       BVH 動作（BvhSource）：BVH 目錄中的檔案。~/play_bvh
//   policy    policy（PolicySource）：policy 庫中的設定檔。~/run_policy
// 所有入口都把要求交給 PlaybackArbiter，由它以同一套規則（on_same、不可中斷、preempt）決定能不能
// 取代目前在播的，所以四種入口可以互相取代。action 的共同行為在 PlaybackServer。
//
// Thread：
//   RT（update()）          讀取 planner 的輸出、寫入介面、更新 state；不碰任何內容庫、不建立或釋放 Motion
//   planner（non-RT）       Motion 的 on_enter() / update()
//   播放用的 callback group 會載入內容的 action（編舞、BVH、policy）與所有 service：讀檔、解析、建立演算法實例，
//                           然後送交 arbiter。載入完成才切換，載入期間（以及載入失敗時）機器人繼續原本的動作
//   預設的 callback group   ~/switch_mode、~/mode、~/cmd_vel、等待播完的 timer：不做載入，所以不會被載入擋住
//                           （停止不必排在別人的載入後面）
//
// 參數：
//   command_interfaces       下游 reference interface（例如 left_arm_group_controller/L_SHOULDER_P）
//   sample_period_interfaces （選填）下游的 sample_period reference（例如 left_arm_group_controller/sample_period），
//                            每週期寫入目前 Motion 的輸出點間隔，供下游插補
//   velocity_command_interfaces / kp_command_interfaces / kd_command_interfaces
//                            （選填）與 command_interfaces 一一對應的下游速度前饋與 PD 增益 reference
//                            （例如 left_arm_group_controller/L_SHOULDER_P/kp），下游為 PdController 時設定。
//                            每週期寫入 Motion 的輸出，沒有指定的值為 NaN（下游使用預設值）
//   state_interfaces         要讀取並發布的 state interface
//   sensor                   （選填）sensor state interface，會自動加入 state_interfaces
//   state_publish_rate       ~/states 的發布頻率 (Hz)
//   target_state_interfaces  與 command_interfaces 一一對應的 state interface，activate 時作為
//                            planner 的起點，也是交給 Motion 的量測角度；
//                            其 prefix（最後一個 '/' 之前）即為該介面的 joint 名稱
//   velocity_state_interfaces（選填）與 command_interfaces 一一對應的速度 state interface，
//                            交給 Motion 的量測速度；空的時候為 NaN。會自動加入 state_interfaces
//   imu_sensor_name          （選填）IMU sensor 名稱（例如 imu_sensor），讀取 <name>/orientation.x …
//                            linear_acceleration.z 共 10 個 state interface 交給 Motion，
//                            會自動加入 state_interfaces
//   planner_rate             planner 的規劃頻率 (Hz)
//   planner_max_steps        每個 command interface 在每個規劃週期最多移動的量（預設值）
//   base_estimator.plugin    （選填）graphen_locomotion::BaseStateEstimator 的 plugin 類別名稱；
//                            其餘 base_estimator.* 參數由估測器自行讀取。空字串表示不估測（base.valid 為 false）
//   command_timeout          ~/cmd_vel 的逾時（秒）；超過時速度指令視為 0（command.active 為 false）
//   switch_mode_poll_rate    檢查「等待播完」的 goal 的頻率 (Hz)，也是 feedback 的頻率（所有播放類 action 共用）
//   robot_description        （BVH 必要、編舞的限位檢查需要）URDF，由 launch 傳給 controller_manager
//
//   系統模式：
//   modes                    系統模式的名稱，同時作為下面各模式參數的前綴（只允許 [A-Za-z0-9_]；
//                            choreo / bvh / policy 是保留名稱）
//   <mode>.type              target（預設）、hold 或 plugin
//   <mode>.max_steps         （選填）該模式的每週期最大移動量，預設為 planner_max_steps
//   <mode>.interruptible     （選填，預設 true）false 時，這個模式播放中（尚未結束）拒絕被其他播放取代
//                            或重播，只有 preempt 的模式可以切入；播完或失敗之後恢復可切換
//   <mode>.preempt           （選填，hold 預設 true，其他預設 false）true 時可以切入不可中斷的播放，
//                            用於停止類的模式（安全底線）
//   type: target
//     <mode>.joints          （選填）要移動的 joint；省略時為全部，依 command_interfaces 順序
//     <mode>.targets         對應 joints 的目標位置；沒有列出的 joint 保持進入模式時的位置
//   type: hold               保持進入模式時的位置（停止），沒有其他參數
//   type: plugin
//     <mode>.plugin          graphen_locomotion::LocomotionAlgorithm 的 plugin 類別名稱；
//                            其餘 <mode>.* 參數由演算法自行讀取
//
//   內容與 policy（各自的參數見 ChoreoSource / BvhSource / PolicySource）：
//   choreo.plugin / choreo.library_path / choreo.max_steps / 其餘 choreo.*
//   bvh.library_path / bvh.max_steps / bvh.approach_max_steps / bvh.interruptible / bvh.base_link …
//   policy.library_path；<policy 名稱>.<鍵> 覆寫該 policy 設定檔中的同名鍵
//
//   啟動時的動作（兩者只能擇一；都是空字串時保持目前位置，直到收到第一個播放指令）：
//   default_mode             系統模式名稱
//   default_policy           policy 名稱；configure 時就載入，設定有誤會在啟動時發現
//
// 介面：
//   ~/mode           (std_msgs/String, 訂閱)                 切換系統模式；未知模式會被忽略
//   ~/cmd_vel        (geometry_msgs/Twist, 訂閱)             機身速度指令（linear.x、linear.y、angular.z）
//   ~/switch_mode    (graphen_interfaces/action/SwitchMode)  切換系統模式
//   ~/play_choreo    (graphen_interfaces/action/PlayChoreo)  播放編舞（choreo.plugin 有設定時）
//   ~/play_bvh       (graphen_interfaces/action/PlayBvh)     播放 BVH（bvh.library_path 有設定時）
//   ~/run_policy     (graphen_interfaces/action/RunPolicy)   執行 policy（policy.library_path 有設定時）
//   ~/publish_choreo / ~/list_choreos / ~/get_choreo         舞曲庫的發佈、列出、讀回（service）
//   ~/preload        (graphen_interfaces/srv/Preload)        事先載入 BVH / policy
//   ~/states         (control_msgs/DynamicJointState)        發布所有 state interface
// 四個 action 的 goal 都有 PlaybackOptions（on_same、wait_until_finished），result 附目前的模式、
// 項目與 state 快照。
class SwitchGroupController : public controller_interface::ControllerInterface {
public:
  using SwitchModeAction = graphen_interfaces::action::SwitchMode;
  using PlayChoreoAction = graphen_interfaces::action::PlayChoreo;
  using PlayBvhAction = graphen_interfaces::action::PlayBvh;
  using RunPolicyAction = graphen_interfaces::action::RunPolicy;

  SwitchGroupController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // 系統模式：controllers.yaml 的 modes 列出的具名模式，configure 時建立
  struct SystemMode {
    std::string name;
    std::shared_ptr<Motion> motion;
    PlaybackPolicy policy;
  };

  // 依名稱取得系統模式；未知時回傳 nullptr。表在 configure 後即唯讀，可在任何 thread 查詢
  const SystemMode * find_mode(const std::string & name) const;
  // 系統模式的播放要求
  static PlaybackRequest request_for(const SystemMode & mode);

  // 讀取 <mode>.* 參數並建立該系統模式；設定錯誤時丟出例外
  SystemMode create_mode(const std::string & mode);

  // 建立內容來源與各個 action / service（configure 的後半段）；設定錯誤時丟出例外
  void create_playback_interfaces();

  // 播放類 action 共用的 hooks（active、fill_state）
  template<typename ActionT>
  typename PlaybackServer<ActionT>::Hooks common_hooks();

  // 以 state_interfaces_ 的目前值更新量測狀態（不配置記憶體，RT 可呼叫）
  void read_robot_state(RobotState & state) const;

  // planner thread 每週期呼叫：取得量測狀態，補上速度指令，並執行機身狀態估測器
  void fill_planner_state(RobotState & state);

  // 讀取 base_estimator.plugin 並建立、configure 估測器；未設定時回傳 nullptr，設定錯誤時丟出例外
  std::shared_ptr<locomotion::BaseStateEstimator> create_base_estimator();

  // 成員的宣告順序就是相依順序（解構時反過來）：
  //   plugin_factory_（演算法的 loader）必須比所有 Motion 活得久：系統模式、planner 中的、各來源快取的；
  //   planner_ 必須比 arbiter_ 活得久；arbiter_ 與各來源必須比 action server 活得久
  PluginFactory plugin_factory_;
  std::unique_ptr<pluginlib::ClassLoader<locomotion::BaseStateEstimator>> estimator_loader_;

  std::vector<std::string> command_interface_names_;
  // 速度前饋與 PD 增益的下游介面；沒有設定時為空（下游為位置模式）
  std::vector<std::string> velocity_interface_names_;
  std::vector<std::string> kp_interface_names_;
  std::vector<std::string> kd_interface_names_;
  std::vector<std::string> sample_period_interface_names_;
  std::vector<std::string> state_interface_names_;

  // 各 command interface 對應的 joint 名稱（取自 target_state_interfaces）
  std::vector<std::string> joint_names_;
  std::vector<double> default_max_steps_;  // planner_max_steps
  double planner_rate_{0.0};

  // 於 on_configure 建立，之後唯讀；Motion 只有 planner thread 使用
  std::vector<SystemMode> system_modes_;
  // activate 時開始播放的要求（default_mode / default_policy）；沒有設定時為空，保持目前位置
  std::optional<PlaybackRequest> default_request_;

  std::string sensor_interface_name_;
  std::optional<size_t> sensor_index_;  // sensor 在 state_interfaces_ 中的 index
  // 最新的 sensor 讀值，供後續控制邏輯使用
  double sensor_value_{std::numeric_limits<double>::quiet_NaN()};

  // target_state_interfaces 在 state_interfaces_ 中的 index，順序對應 command_interfaces
  std::vector<size_t> target_state_indices_;
  // velocity_state_interfaces 在 state_interfaces_ 中的 index；沒有設定時為空
  std::vector<size_t> velocity_state_indices_;
  // IMU 10 個 state interface 在 state_interfaces_ 中的 index（orientation.xyzw、
  // angular_velocity.xyz、linear_acceleration.xyz）；沒有設定時為空
  std::vector<size_t> imu_state_indices_;

  // update()（RT）以 try_lock 寫入量測狀態，planner thread 每週期讀取
  TryLockBuffer<RobotState> state_buffer_;

  // 機身狀態估測器；以下狀態在 activate 時設定，之後只在 planner thread 中存取
  // （planner thread 在設定之後才啟動，不需要額外同步）
  std::shared_ptr<locomotion::BaseStateEstimator> base_estimator_;
  bool estimator_needs_reset_{true};
  bool estimator_faulted_{false};
  double last_state_time_{0.0};

  // ~/cmd_vel 的最新指令：subscription callback 寫入、planner thread 讀取，兩者都不是 RT
  struct VelocityCommand {
    std::array<double, 3> velocity{0.0, 0.0, 0.0};  // vx, vy, wz
    std::chrono::steady_clock::time_point stamp;
    bool received{false};
  };
  std::mutex command_mutex_;
  VelocityCommand velocity_command_;
  double command_timeout_{0.5};

  // planner（non-RT thread）寫入，update() 從 RT 端讀取
  std::unique_ptr<LocomotionPlanner> planner_;
  LocomotionPlanner::TargetBuffer target_buffer_;

  // update() 以 try_lock 寫入最新的 state 值，供 action result 讀取快照
  std::mutex state_snapshot_mutex_;
  std::vector<double> state_snapshot_;

  // 將所有 state interface 發布至 ~/states
  std::unique_ptr<StateInterfacePublisher> state_publisher_;

  // 切換規則；所有入口共用。planner_ 重建時一起重建
  std::unique_ptr<PlaybackArbiter> arbiter_;

  // 內容與 policy 的來源（各自的庫與快取）；沒有啟用的類型仍會建立，enabled() 為 false
  std::unique_ptr<ChoreoSource> choreo_source_;
  std::unique_ptr<BvhSource> bvh_source_;
  std::unique_ptr<PolicySource> policy_source_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscriber_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_{nullptr};

  // 會載入內容的 action（編舞、BVH、policy）與 service 專用的 callback group：載入在這裡進行，
  // 不會擋住預設 group 中的 ~/switch_mode、~/cmd_vel、~/mode 與 wait_timer_
  rclcpp::CallbackGroup::SharedPtr playback_group_{nullptr};
  std::vector<std::unique_ptr<PlaybackServerBase>> playback_servers_;
  rclcpp::Service<graphen_interfaces::srv::PublishChoreo>::SharedPtr publish_choreo_service_;
  rclcpp::Service<graphen_interfaces::srv::ListChoreos>::SharedPtr list_choreos_service_;
  rclcpp::Service<graphen_interfaces::srv::GetChoreo>::SharedPtr get_choreo_service_;
  rclcpp::Service<graphen_interfaces::srv::Preload>::SharedPtr preload_service_;
  // 定時呼叫各 server 的 poll()（等待播完的 goal）
  rclcpp::TimerBase::SharedPtr wait_timer_{nullptr};
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__SWITCH_GROUP_CONTROLLER_HPP_
