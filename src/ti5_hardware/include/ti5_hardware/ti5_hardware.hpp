#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "can_driver/can_manager.hpp"
#include "ti5_hardware/joint_safety_control.hpp"

#include <yaml-cpp/yaml.h>

namespace ti5_hardware
{

class Ti5Hardware : public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Ti5Hardware)

        /**
        * @brief 匯出狀態介面（會被　resource_manager　呼叫，用於設定 joint 的狀態、速度、命令）
        * @return 狀態介面
        */
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /**
        * @brief 匯出命令介面（會被　resource_manager　呼叫，用於設定 joint 的狀態、速度、命令）
        * @return 命令介面
        */
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /**
        * @brief hardware_interface step1 : 初始化
        * @param info 硬體資訊
        * @return 初始化結果
        */
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        /**
        * @brief hardware_interface step2 : 配置（初始化 CAN manager，檢測 URDF 中的 joint 與 CAN 設定是否一致）
        * @param previous_state 上一個狀態
        * @return 配置結果
        */
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        /**
        * @brief hardware_interface step3 : 啟動（讀取 CAN 並更新當前馬達的狀態）
        * @param previous_state 上一個狀態
        * @return 啟動結果
        */
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        /**
        * @brief hardware_interface step4 : 停止 CAN 讀取
        * @param previous_state 上一個狀態
        * @return 停用結果
        */
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        /**
        * @brief hardware_interface step5 : 停止 CAN 讀取
        * @param previous_state 上一個狀態
        * @return 清理結果
        */
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        /**
        * @brief hardware_interface : 讀取 CAN 並更新當前馬達的狀態
        * @param time 時間
        * @param period 週期
        * @return 讀取結果
        */
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        /**
        * @brief hardware_interface : 寫入 CAN 並更新當前馬達的狀態
        * @param time 時間
        * @param period 週期
        * @return 寫入結果
        */
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        /**
        * @brief 指令模式，由 URDF 的 command interface 決定（所有 joint 必須一致）
        *
        * kPosition：0x1E 位置模式（hardware:=real）
        * kEffort：0x1C 電流模式，力矩依 KT 與減速比換算（hardware:=real_pd，PD 在 controller）
        */
        enum class ControlMode { kPosition, kEffort };

        /**
        * @brief 一條 CAN 通道與其馬達對應的 joint（effort 模式批次收發用）
        */
        struct ChannelGroup {
            std::shared_ptr<can_driver::CanDriverBuilder> builder;
            /// 馬達索引 → joint 索引；沒有對應 joint 的馬達為 kNoJoint
            std::vector<size_t> joint_of_motor;
        };
        static constexpr size_t kNoJoint = std::numeric_limits<size_t>::max();

        hardware_interface::return_type readPositionMode();
        hardware_interface::return_type writePositionMode(const rclcpp::Duration & period);
        hardware_interface::return_type readEffortMode();
        hardware_interface::return_type writeEffortMode();

        /**
        * @brief 所有馬達切回 0x1E 位置模式，保持在最後量測的位置（effort 模式的故障退回）
        *
        * 退回後不再輸出力矩，直到 lifecycle reset（deactivate → activate）。
        * 不用去使能，避免機器人直接軟倒。
        */
        void enterPositionHold(const char * reason);

        ControlMode control_mode_{ControlMode::kPosition};

        /**
        * @brief can config 路徑
        */
        std::string can_config_path_;

        YAML::Node initial_positions_;

        /**
        * @brief CAN 準備狀態
        */
        bool is_can_ready_{false};

        /**
        * @brief 命令無效故障（關節命令為 NaN / inf，無法 clamp）
        *
        * 一旦觸發，系統會拒絕後續 read/write，直到透過 lifecycle reset（例如 deactivate->activate）
        * 清除此狀態。超出限位的有限命令不會觸發，而是 clamp 到限位後輸出。
        */
        bool is_command_invalid_{false};

        /**
        * @brief 各關節目前是否正在被 clamp（只用於在狀態轉換時印 log）
        */
        std::vector<bool> is_clamping_;

        /**
        * @brief CAN 讀取超時時間閥值（ms）
        */
        int hw_read_timeout_ms_;

        /**
        * @brief 送往馬達的指令最大變化速度（rad/s），<= 0 表示不限速
        *
        * write() 不直接送出 hw_commands_，而是讓 hw_output_commands_ 每週期最多往
        * hw_commands_ 前進 max_command_velocity * dt，開機走向初始姿態時也會受限。
        */
        double max_command_velocity_{0.0};
        /**
        * @brief 實際送往馬達的指令（弧度），activate 時以 encoder 位置為起點
        */
        std::vector<double> hw_output_commands_;
        /**
        * @brief 與 URDF 一致的 joint 名稱順序（僅包含有對應馬達的 joint）
        */
        std::vector<std::string> joint_names_;

        /**
        * @brief 每個 joint 對應的 channel 名稱（與 joint_names_ 同序）
        */
        std::vector<std::string> joint_to_channel_;

        /**
        * @brief 每個 joint 在該 channel 內的馬達索引（0 = 該通道 can_id 第一顆）
        */
        std::vector<size_t> joint_to_motor_index_;

        /**
        * @brief 關節安全控制（讀取/保存限位）
        */
        JointSafetyControl joint_safety_control_;

        /**
        * @brief 每個 joint 的命令（弧度）
        */
        std::vector<double> hw_commands_;

        /**
        * @brief 每個 joint 的狀態（弧度）
        */
        std::vector<double> hw_states_;

        /**
        * @brief 每個 joint 的速度（弧度/秒）（若無編碼器速度可填 0）
        */
        std::vector<double> hw_velocities_;

        /**
        * @brief 每個 joint 的濾波電流（mA）
        */
        std::vector<double> hw_filter_currents_;
        /**
        * @brief 每個 joint 的錯誤碼
        */
        std::vector<double> hw_error_codes_;

        /**
        * @brief effort 模式：每個 joint 的力矩命令（Nm，輸出端）；controller 寫入前為 NaN
        */
        std::vector<double> hw_effort_commands_;

        /**
        * @brief effort 模式：由回饋電流估計的輸出端力矩（Nm）= 電流 × KT × 減速比
        */
        std::vector<double> hw_efforts_;

        /**
        * @brief 每個 joint 的馬達扭矩常數（電機端 Nm/A，configure 時從馬達讀取；只有 effort 模式使用）
        */
        std::vector<double> torque_constants_;

        /**
        * @brief 每個 joint 的減速比（configure 時從馬達讀取）
        */
        std::vector<double> reduction_ratios_;

        /**
        * @brief effort 模式的 CAN 通道分組與預先配置的回饋緩衝（read() 不配置記憶體）
        */
        std::vector<ChannelGroup> channel_groups_;
        std::vector<can_driver::ChannelFeedback> channel_feedback_;

        /**
        * @brief effort 模式：所有 joint 都收到有限的力矩命令後才切到電流模式，之前維持位置保持
        */
        bool torque_active_{false};
        bool prev_torque_active_{false};  // 追蹤 torque_active_ 轉換，切換瞬間重設 missed_cycles_

        /**
        * @brief effort 模式：已退回位置保持（見 enterPositionHold）
        */
        bool position_hold_fallback_{false};

        /**
        * @brief effort 模式：容許連續回饋缺漏的週期數，超過就退回位置保持
        */
        int max_missed_cycles_{0};

        /**
        * @brief effort 模式：目前連續回饋缺漏的週期數
        */
        int missed_cycles_{0};

        /**
        * @brief can manager 指標
        */
        std::shared_ptr<can_driver::CanManager> can_manager_;

        /**
        * @brief 每個 joint 對應的 builder 快取（與 joint_names_ 同序）
        */
        std::vector<std::shared_ptr<can_driver::CanDriverBuilder>> joint_builders_;

        /**
        * @brief 連續「read() 因硬體讀取失敗而回 ERROR」的次數（用於 log：首錯詳細、其餘只計數、恢復時 WARN）
        */
        int consecutive_read_fail_cycles_{0};
};

}  // namespace ti5_hardware

