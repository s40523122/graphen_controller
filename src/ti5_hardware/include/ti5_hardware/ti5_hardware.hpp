#pragma once

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
        * @brief can config 路徑
        */
        std::string can_config_path_;

        YAML::Node initial_positions_;

        /**
        * @brief CAN 準備狀態
        */
        bool is_can_ready_{false};

        /**
        * @brief 超限故障（例如：關節命令超出限位）
        *
        * 一旦觸發，系統會拒絕後續 read/write，直到透過 lifecycle reset（例如 deactivate->activate）
        * 清除此狀態。
        */
        bool is_over_limit_{false};

        /**
        * @brief CAN 讀取超時時間閥值（ms）
        */
        int hw_read_timeout_ms_;

        /**
        * @brief 關節限位判定容差（弧度）
        */
        double limit_epsilon_rad_{0.0};

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

