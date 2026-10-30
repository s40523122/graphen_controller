#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

// ComponentInfo / InterfaceInfo
#include "hardware_interface/hardware_info.hpp"

namespace rclcpp
{
class Logger;
}

namespace ti5_hardware
{

class JointSafetyControl {
    public:
        /**
        * @brief 關節位置上下限（弧度），與 joint 順序一致
        */
        using LimitPair = std::pair<double, double>;

        /**
        * @brief 建構式
        */
        JointSafetyControl() = default;

        /**
        * @brief 設定每個關節的位置上下限（弧度），與 joint 順序一致
        */
        void setLimits(const std::vector<LimitPair> & limits);

        /**
        * @brief 從 ros2_control + URDF 讀取關節限位並設定
        *
        * joints_in_order 的順序會直接決定 limits 的順序（需與 Ti5Hardware 的 joint_names_ 一致）。
        *
        * 優先順序：
        * - ros2_control command_interface(position) 的 min/max
        * - ros2_control joint.parameters["min"/"max"]
        * - URDF <joint><limit lower/upper>
        * 力矩上限只取 URDF <limit effort>（effort 模式才需要，由呼叫端檢查是否 > 0）
        *
        * @param urdf_xml URDF 內容（robot_description，即 HardwareInfo::original_xml）
        * @throws std::runtime_error URDF 解析失敗，或 joint 沒有任何限位來源
        */
        void setLimitsFromJoints(const std::vector<hardware_interface::ComponentInfo> & joints_in_order,
                                 const std::string & urdf_xml,
                                 const rclcpp::Logger & logger);

        /**
        * @brief 取得第 index 個關節的上下限（弧度）
        * @param index 關節索引
        * @param min_rad_out 輸出最小值（弧度）
        * @param max_rad_out 輸出最大值（弧度）
        * @throws std::out_of_range 當 index >= size()
        */
        void getLimit(size_t index, double & min_rad_out, double & max_rad_out) const;

        /**
        * @brief 將命令夾在第 index 個關節的限位內（弧度）
        *
        * command_rad 必須是有限值；NaN / inf 由呼叫端先行處理。
        * @throws std::out_of_range 當 index >= size()
        */
        double clamp(size_t index, double command_rad) const;

        /**
        * @brief 第 index 個關節的力矩上限（Nm，URDF <limit effort>；未設定時為 0）
        * @throws std::out_of_range 當 index >= size()
        */
        double effortLimit(size_t index) const;

        /**
        * @brief 已設定的關節數量
        */
        size_t size() const noexcept { return limits_.size(); }

        /**
        * @brief 是否已設定限位（size() > 0）
        */
        bool empty() const noexcept { return limits_.empty(); }

    private:
        /**
        * @brief 關節位置上下限（弧度），與 joint 順序一致
        */
        std::vector<LimitPair> limits_;

        /**
        * @brief 關節力矩上限（Nm），與 joint 順序一致
        */
        std::vector<double> effort_limits_;
};

}  // namespace ti5_hardware
