#pragma once

#include <chrono>
#include <cstdint>
#include <string>

// Linux CAN header
#include <linux/can.h>
#include <sys/time.h>

namespace can_driver
{

class SocketCanInterface
{
    public:
        /**
        * @brief 建構式：從 config 開啟指定名稱的單一 CAN 通道
        * @param channel_name 通道名稱，如 "can0", "can1"
        */
        explicit SocketCanInterface(const std::string & channel_name);

        /**
        * @brief 解構式
        */
        ~SocketCanInterface();

        SocketCanInterface(const SocketCanInterface &) = delete;
        SocketCanInterface & operator=(const SocketCanInterface &) = delete;

        SocketCanInterface(SocketCanInterface &&) = delete;
        SocketCanInterface & operator=(SocketCanInterface &&) = delete;

        /**
        * @brief 嘗試開啟並綁定 CAN 介面
        * @return true：成功，false：失敗
        */
        bool open();

        /**
        * @brief 關閉 socket
        */
        void close();

        /**
        * @brief 檢查 socket 是否已開啟
        * @return true：已開啟，false：未開啟
        */
        bool isOpen() const noexcept;

        /**
        * @brief 傳送一個 CAN frame
        * @param can_id CAN ID
        * @param data 資料
        * @param dlc 資料長度
        * @return true：成功，false：失敗
        */
        bool sendFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc);

        /**
        * @brief 傳送已填好的 frame
        * @param frame CAN frame
        * @return true：成功，false：失敗
        */
        bool sendFrame(const struct can_frame & frame);

        /**
        * @brief 阻塞接收一個 frame，timeout_ms < 0 代表無限等待
        * @param frame CAN frame
        * @param timeout_ms 超時時間，< 0 代表無限等待
        * @return true：成功，false：失敗
        */
        bool receiveFrame(struct can_frame & frame, int timeout_ms);

        /**
        * @brief 阻塞接收一個 frame，timeout 為微秒精度（200 Hz 迴圈內需要小於 1 ms 的等待）
        * @param frame CAN frame
        * @param timeout 超時時間，<= 0 代表只檢查目前是否有資料（不等待）
        * @return true：成功，false：超時或失敗
        */
        bool receiveFrame(struct can_frame & frame, std::chrono::microseconds timeout);

        /**
        * @brief 取得通道名稱
        * @return 通道名稱
        */
        const std::string & getChannelName() const noexcept { return channel_; }

    private:
        /**
        * @brief 通道名稱
        */
        std::string channel_;

        /**
        * @brief CAN 介面的連線代號 (File Descriptor, FD)
        * @note 有開時是有效 fd，用來送/收 CAN；關掉時被 close 並設成 -1。
        */
        int sock_fd_;

        /**
        * @brief select 等待後讀取一個 frame；timeout 為 nullptr 代表無限等待
        */
        bool waitAndRead(struct can_frame & frame, struct timeval * timeout);
};

}  // namespace can_driver

