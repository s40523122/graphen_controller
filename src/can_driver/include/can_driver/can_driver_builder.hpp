#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "can_driver/socketcan_interface.hpp"
#include "can_driver/motor_control.hpp"

namespace can_driver
{

/// 一次批次收發的結果（requestFeedbackAll → collectFeedbackAll）
struct ChannelFeedback {
    /// 與馬達索引同序；沒收到回覆的為 std::nullopt
    std::vector<std::optional<MotorFeedback>> motors;
    /// 本輪輪詢到的錯誤碼（馬達索引, 錯誤碼）；本輪沒輪詢或沒收到時為 std::nullopt
    std::optional<std::pair<size_t, uint32_t>> error;
};

class CanDriverBuilder {
    public:

        /**
        * @brief 建構式：從 config 開啟指定名稱的單一 CAN 通道
        * @param config_path can config 路徑
        * @param channel_name 通道名稱，如 "left_leg"
        */
        CanDriverBuilder(const std::string & config_path, const std::string & channel_name);

        /**
        * @brief 解構式
        */
        ~CanDriverBuilder();

        /**
        * @brief 從 config_path 讀取參數並建立對應 dof_ 的馬達數量
        * @return true：參數讀取與馬達建立成功，false 失敗
        */
        bool initialize();

        /**
        * @brief 取得 SocketCAN 介面指標
        */
        std::shared_ptr<SocketCanInterface> getSocketCan() const noexcept { return socketCan_; }

        /**
        * @brief 取得第 index 顆馬達（0-based），index 對應該通道內順序，如 left_leg: 0=L_HIP_P, 1=L_HIP_R, ...
        * @param index 馬達索引，須 0 <= index < dof()
        * @return 馬達指標，越界回傳 nullptr
        */
        std::shared_ptr<MotorControl> getMotor(size_t index) const;

        /**
        * @brief 取得第一顆馬達（向後相容）
        */
        std::shared_ptr<MotorControl> getMotor() const noexcept { return getMotor(0); }

        /**
        * @brief 取得馬達數量（該通道的 dof）
        */
        int dof() const noexcept { return dof_; }

        /**
        * @brief 取得此通道第一顆馬達的 CAN ID（config 中的 can_id）
        */
        uint16_t can_id() const noexcept { return can_id_; }

        /**
        * @brief 讀第 motor_index 顆馬達目前位置（cnt / rad）
        * @param motor_index 馬達索引，須 0 <= motor_index < dof()
        * @param out_cnt 輸出位置計數
        * @param out_rad 輸出位置弧度
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false 失敗
        */
        bool readPosition(size_t motor_index, int32_t & out_cnt, double & out_rad, int timeout_ms);

        /**
        * @brief 讀第一顆馬達位置（向後相容）
        * @param out_cnt 輸出位置計數
        * @param out_rad 輸出位置弧度
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false 失敗
        */
        bool readPosition(int32_t & out_cnt, double & out_rad, int timeout_ms);

        /**
        * @brief 第 motor_index 顆馬達相對移動（單位：rad）
        * @param motor_index 馬達索引，須 0 <= motor_index < dof()
        * @param delta_rad 相對移動弧度
        * @param timeout_ms_for_readback 讀取超時時間，單位：毫秒
        * @return true：移動成功，false 失敗
        */
        bool moveRelativeRad(size_t motor_index, double delta_rad, int timeout_ms_for_readback);

        /**
        * @brief 第一顆馬達相對移動（向後相容）
        * @param delta_rad 相對移動弧度
        * @param timeout_ms_for_readback 讀取超時時間，單位：毫秒
        * @return true：移動成功，false 失敗
        */
        bool moveRelativeRad(double delta_rad, int timeout_ms_for_readback);

        /**
        * @brief 第 motor_index 顆馬達絕對移動（單位：rad）
        * @param motor_index 馬達索引，須 0 <= motor_index < dof()
        * @param target_rad 目標弧度
        * @return true：移動成功，false 失敗
        */
        bool moveToPositionRad(size_t motor_index, double target_rad);

        /**
        * @brief 第一顆馬達絕對移動（向後相容）
        * @param target_rad 目標弧度
        * @return true：移動成功，false 失敗
        */
        bool moveToPositionRad(double target_rad);

        /**
        * @brief 讀第 motor_index 顆馬達濾波電流（mA）
        * @param motor_index 馬達索引，須 0 <= motor_index < dof()
        * @param out_ma 輸出濾波電流 (mA)
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false 失敗
        */
        bool readFilterCurrent(size_t motor_index, double & out_ma, int timeout_ms);

        /**
        * @brief 讀第 motor_index 顆馬達錯誤碼
        * @param motor_index 馬達索引，須 0 <= motor_index < dof()
        * @param out_error 輸出錯誤碼
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false 失敗
        */
        bool readError(size_t motor_index, uint32_t & out_error, int timeout_ms);

        /**
        * @brief 讀第一顆馬達錯誤碼（向後相容）
        * @param out_error 輸出錯誤碼
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false 失敗
        */
        bool readError(uint32_t & out_error, int timeout_ms);

        /**
        * @brief 清除錯誤
        * @return true：讀取成功，false 失敗
        */
        bool clearError(size_t motor_index);

        /**
        * @brief 讀第 motor_index 顆馬達目前速度（單位：rad/s）
        * @param motor_index 馬達索引，須 0 <= motor_index < dof()
        * @param out_rad_per_sec 輸出角速度（弧度/秒）
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false 失敗
        */
        bool readSpeed(size_t motor_index, double & out_rad_per_sec, int timeout_ms);

        /**
        * @brief 讀第一顆馬達速度（向後相容，單位：rad/s）
        * @param out_rad_per_sec 輸出角速度（弧度/秒）
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false 失敗
        */
        bool readSpeed(double & out_rad_per_sec, int timeout_ms);

        /**
        * @brief 丟棄 socket 中尚未讀取的 frame（上一輪逾時後才到的回覆），避免與本輪混淆
        */
        void drainReceived();

        /**
        * @brief 對本通道所有馬達送出 0x41（取得電流速度位置），不等待回覆
        *
        * 輪詢錯誤碼時另外對一顆馬達送出 0x0A，每次呼叫輪到下一顆（round-robin），
        * 讓錯誤碼不必每週期查詢所有馬達。
        * @param poll_error 是否輪詢錯誤碼
        * @return true：全部送出成功，false 任一送出失敗
        */
        bool requestFeedbackAll(bool poll_error);

        /**
        * @brief 收集 requestFeedbackAll 的回覆，直到全部收到或到達 deadline
        * @param deadline 截止時間（多個通道共用同一個 deadline，讓各通道的等待重疊）
        * @param out 結果；out.motors 會被重設為 dof() 個元素
        * @return true：全部回覆都收到，false：有回覆缺漏
        */
        bool collectFeedbackAll(std::chrono::steady_clock::time_point deadline, ChannelFeedback & out);

        /**
        * @brief 收取 socket buffer 中所有已到達的 8-byte feedback（不等待）。
        *        與 setTargetCurrentMaWithFeedback 搭配：write 送 0x42，下一個 read 週期呼叫此方法。
        * @param out 結果；out.motors 會被重設為 dof() 個元素
        */
        void drainAndCollectFeedback(ChannelFeedback & out);

    private:
        /**
        * @brief config 檔案路徑
        */
        std::string config_file_;

        /**
        * @brief channel 是否啟用
        */
        bool channel_enable_;

        /**
        * @brief 通道名稱
        */
        std::string channel_name_;

        /**
        * @brief can 通道
        */
        std::string channel_;

        /**
        * @brief 傳輸速率
        */
        int bitrate_;

        /**
        * @brief 傳輸佇列長度
        */
        int txqueuelen_;

        /**
        * @brief 紀錄 CAN 通道的初始值 ID
        */
        uint16_t can_id_;

        /**
        * @brief 紀錄每一個 can 通道的馬達數量
        */
        int dof_;

        /**
        * @brief 雙精度 Encoder 外圈 CPR
        */
        double encoder_dual_outer_;
        /**
        * @brief 單精度 Encoder 內圈 CPR
        */
        double encoder_single_inner_;

        /**
        * @brief SocketCAN 介面指標
        */
        std::shared_ptr<SocketCanInterface> socketCan_;

        /**
        * @brief 此通道上的馬達控制物件（dof_ 顆），ID 為 can_id_+0, can_id_+1, ...
        */
        std::vector<std::shared_ptr<MotorControl>> motors_;

        /**
        * @brief 下一次輪詢錯誤碼的馬達索引
        */
        size_t error_poll_index_{0};

        /**
        * @brief 本輪正在等待回覆的錯誤碼馬達索引
        */
        std::optional<size_t> pending_error_index_;
    };
}  // namespace can_driver

