#pragma once

#include <cstddef>
#include <optional>
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
        * @brief 從 ros2_control + (可選)URDF 讀取關節限位並設定
        *
        * joints_in_order 的順序會直接決定 limits 的順序（需與 Ti5Hardware 的 joint_names_ 一致）。
        *
        * 優先順序：
        * - ros2_control command_interface(position) 的 min/max
        * - ros2_control joint.parameters["min"/"max"]
        * - URDF <joint><limit lower/upper>（若 urdf_path 有提供且讀取成功）
        * - 預設 [-pi, pi]
        */
        void setLimitsFromJoints(const std::vector<hardware_interface::ComponentInfo> & joints_in_order,
                                 const std::optional<std::string> & urdf_path,
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
        * @brief 檢查單一關節命令是否在限位內（弧度）
        * @throws std::out_of_range 當 index >= size()
        */
        bool isCommandWithinLimits(size_t index, double command_rad) const;

        /**
        * @brief 檢查一組命令是否全都在限位內（弧度）
        * @throws std::out_of_range 當 commands 數量或索引與 limits 不一致
        */
        bool areCommandsWithinLimits(const double * commands, size_t n) const;

        /**
        * @brief 設定限位比較容差（弧度）
        */
        void setLimitEpsilon(double epsilon_rad) noexcept { limit_epsilon_rad_ = epsilon_rad; }

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
        * @brief 限位比較容差（弧度）
        */
        double limit_epsilon_rad_{0.0};
};

}  // namespace ti5_hardware
