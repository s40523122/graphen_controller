#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "can_driver/can_driver_builder.hpp"

namespace can_driver
{

class CanManager
{
    public:
        /**
        * @brief 建構式（不執行 initialize，需另呼叫 initialize()）
        * @param config_file can config 路徑
        */
        explicit CanManager(const std::string & config_file);

        /**
        * @brief 初始化：依 config 中 "CAN" 底下的每個通道名稱，建立並初始化對應的 CanDriverBuilder
        * @return true 成功，false 失敗
        */
        bool initialize();

        /**
        * @brief 關閉並釋放所有 channel builder（讓 SocketCAN / MotorControl 依解構釋放資源）。
        *
        * 注意：此函式不會丟例外，呼叫後所有 builder 都會被移除。
        * 之後若要再次使用，請重新呼叫 initialize()。
        */
        void shutdown() noexcept;

        /**
        * @brief 依通道名稱取得 builder
        * @param channel_name config 中 "CAN" 底下的通道名稱，如 "left_leg"
        * @return 對應的 builder，無則回傳 nullptr
        */
        std::shared_ptr<CanDriverBuilder> getBuilderPtr(const std::string & channel_name) const;

        /**
        * @brief 取得目前已建立的所有通道名稱（依名稱排序）
        */
        std::vector<std::string> channelNames() const;

    private:
        /**
        * @brief config 檔路徑
        */
        std::string config_file_;

        /**
        * @brief 通道名稱 -> builder；用 std::map 讓初始化與 log 順序固定，方便除錯
        */
        std::map<std::string, std::shared_ptr<CanDriverBuilder>> builders_;
};
}  // namespace can_driver
