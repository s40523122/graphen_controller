#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include <linux/can.h>

#include "can_driver/socketcan_interface.hpp"

namespace can_driver
{

/// 對應文件 7.1 的一字節指令（只列出常用 + 可擴充）
enum class OneByteCommand : uint8_t {
    DisableMotor = 0x02,
    GetMode = 0x03,
    GetCurrent = 0x04,
    GetFilteredCurrent = 0x05,
    GetSpeed = 0x06,
    GetTargetSpeed = 0x07,
    GetPosition = 0x08,
    GetTargetPosition = 0x09,
    GetErrorStatus = 0x0A,
    ClearError = 0x0B,
    SaveToFlash = 0x0E,
    RestoreFactory = 0x0F,
    GetFeedback = 0x41  // 回覆 8 位元組：電流 int16 (mA)、速度 int16 (0.01 Hz，電機端)、位置 int32 (cnt)
// 如需完整對應，可依文件 7.1 繼續擴充
};

/// 對應文件 7.2 的五字節指令（常用）
enum class FiveByteCommand : uint8_t {
    DisableMotor = 0x02,
    SetTargetCurrent = 0x1C,
    SetTargetSpeed = 0x1D,
    SetTargetPosition = 0x1E,
    SetMaxCurrent = 0x20,
    SetMinCurrent = 0x21,
    SetAccel = 0x22,
    SetDecel = 0x23,
    SetMaxSpeed = 0x24,
    SetMinSpeed = 0x25,
    SetMaxPosition = 0x26,
    SetMinPosition = 0x27,
    SetCanId = 0x2E,
    SetProfileAccel = 0x3B,
    SetProfileJerk = 0x3C,
    SetCurrentWithFeedback = 0x42  // 回覆格式同 OneByteCommand::GetFeedback
    // 其餘指令可依需求擴充
};

/// 對應文件 7.3 的六字節指令（指令碼 + 讀寫旗標 + 4-byte 參數，小端）
enum class SixByteCommand : uint8_t {
    ReductionRatio = 0x41,  // int32
    TorqueConstant = 0x44   // float，電機端 Nm/A
};

/// 六字節指令的讀寫旗標（第 2 位元組）
constexpr uint8_t kSixByteWrite = 0x20;
constexpr uint8_t kSixByteRead = 0x40;

/// 0x41 回覆的原始值（單位為協議單位，換算見 MotorControl）
struct MotorFeedback {
    int16_t current_ma{0};
    int16_t speed_centi_hz{0};
    int32_t position_cnt{0};
};

/// TI5 C1 關節模組的高階封裝，對應 PDF 中的 CAN 指令
class MotorControl
{
    public:
        /**
        * @brief 建構式
        * @param socketCan SocketCAN 介面
        * @param can_id CAN ID
        * @param counts_per_revolution 輸出軸一圈對應的 encoder 計數（例：DualOuter=262144，SingleInner 則為 65536*減速比）
        */
        MotorControl(SocketCanInterface & socketCan, uint16_t can_id, double counts_per_revolution);

        /**
        * @brief 取得 CAN ID
        */
        uint16_t can_id() const noexcept { return can_id_; }

        /**
        * @brief 直接發送一字節指令（無回覆）
        * @param cmd 指令
        * @return true：發送成功，false：發送失敗
        */
        bool sendOneByte(OneByteCommand cmd);

        /**
        * @brief 發送一字節指令並期待 5-byte 回覆（指令碼 + 4-byte 資料）
        * @param cmd 指令
        * @param out_data 輸出資料
        * @param out_dlc 輸出資料長度
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：發送成功，false：發送失敗
        */
        bool sendOneByteWithReply(OneByteCommand cmd,
                                  std::array<uint8_t, 8> & out_data,
                                  uint8_t & out_dlc,
                                  int timeout_ms);

        /**
        * @brief 發送五字節指令（cmd + 4-byte 參數，小端）
        * @param cmd 指令
        * @param param 參數
        * @return true：發送成功，false：發送失敗
        */
        bool sendFiveByte(FiveByteCommand cmd, int32_t param);

        /**
        * @brief 讀取目前位置（cnt）
        * @param out_cnt 輸出位置計數
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false：讀取失敗
        */
        bool readPositionCnt(int32_t & out_cnt, int timeout_ms);

        /**
        * @brief 讀取目前位置（輸出端角度，單位：rad）
        * @param out_rad 輸出位置弧度
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false：讀取失敗
        */
        bool readPositionRad(double & out_rad, int timeout_ms);

        /**
        * @brief 絕對位置移動（cnt）
        * @param target_cnt 目標位置計數
        * @return true：移動成功，false：移動失敗
        */
        bool moveToPositionCnt(int32_t target_cnt);

        /**
        * @brief 絕對位置移動（輸出端角度，單位：rad）
        * @param target_rad 目標位置弧度
        * @return true：移動成功，false：移動失敗
        */
        bool moveToPositionRad(double target_rad);

        /**
        * @brief 相對位置移動（cnt）
        * @param delta_cnt 相對移動計數
        * @param timeout_ms_for_readback 讀取超時時間，單位：毫秒
        * @return true：移動成功，false：移動失敗
        */
        bool moveRelativeCnt(int32_t delta_cnt, int timeout_ms_for_readback);

        /**
        * @brief 相對位置移動（輸出端角度，單位：rad）
        * @param delta_rad 相對移動弧度
        * @param timeout_ms_for_readback 讀取超時時間，單位：毫秒
        * @return true：移動成功，false：移動失敗
        */
        bool moveRelativeRad(double delta_rad, int timeout_ms_for_readback);

        /**
        * @brief 讀取目前速度（輸出端角速度，單位：rad/s）
        * @param out_rad_per_sec 輸出角速度（弧度/秒）
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false：讀取失敗
        */
        bool readSpeedRad(double & out_rad_per_sec, int timeout_ms);

        /**
        * @brief 讀取目前濾波電流（mA）
        * @param out_cnt 輸出電流 (mA)
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false：讀取失敗
        */
        bool readFilterCurrent(double & out_ma, int timeout_ms);

        /**
        * @brief 讀取錯誤碼（對應 0x0A 指令）
        * @param out_error 輸出錯誤碼
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false：讀取失敗
        */
        bool readErrorCode(uint32_t & out_error, int timeout_ms);

        /**
        * @brief 清除錯誤（對應 0x0B 指令）
        * @return true：清除成功，false：清除失敗
        */
        bool clearError();

        /**
        * @brief 設定減速比，用於把電機端速度換算成輸出端（0x06 / 0x41 的速度都是電機端）
        * @param ratio 減速比，須 > 0
        */
        void setReductionRatio(double ratio) noexcept { reduction_ratio_ = ratio; }

        /**
        * @brief 目前使用的減速比
        */
        double reductionRatio() const noexcept { return reduction_ratio_; }

        /**
        * @brief 讀取六字節參數（讀旗標 0x40），回覆的 4-byte 參數原樣輸出
        * @param cmd 指令
        * @param out_raw 回覆的參數（小端原始位元組）
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：讀取成功，false：讀取失敗
        */
        bool readSixByteParam(SixByteCommand cmd, std::array<uint8_t, 4> & out_raw, int timeout_ms);

        /**
        * @brief 讀取馬達內設定的減速比（六字節 0x41）
        */
        bool readReductionRatio(int32_t & out_ratio, int timeout_ms);

        /**
        * @brief 讀取馬達內設定的扭矩常數 KT（六字節 0x44，電機端 Nm/A）
        */
        bool readTorqueConstant(float & out_kt, int timeout_ms);

        /**
        * @brief 設定目標電流（五字節 0x1C，mA）；馬達會切換到電流模式，無回覆
        * @param current_ma 目標電流 (mA)
        * @return true：發送成功，false：發送失敗
        */
        bool setTargetCurrentMa(int32_t current_ma);

        /**
        * @brief 設定目標電流並觸發 feedback（五字節 0x42，mA）；
        *        馬達切換到電流模式，並非同步回覆 8 位元組 feedback（格式同 parseFeedback）。
        *        下一個 read 週期用 drainAndCollectFeedback 收取。
        */
        bool setTargetCurrentMaWithFeedback(int32_t current_ma);

        /**
        * @brief 送出 0x41（取得電流速度位置），不等待回覆；回覆用 parseFeedback 解析
        */
        bool requestFeedback();

        /**
        * @brief 送出 0x0A（取得錯誤碼），不等待回覆；回覆用 parseErrorCode 解析
        */
        bool requestErrorCode();

        /**
        * @brief 解析 0x41 / 0x42 的 8 位元組回覆
        * @return true：frame 是 8 位元組回覆，false：不是
        */
        static bool parseFeedback(const struct can_frame & frame, MotorFeedback & out);

        /**
        * @brief 解析 0x0A 的 5 位元組回覆
        * @return true：frame 是錯誤碼回覆，false：不是
        */
        static bool parseErrorCode(const struct can_frame & frame, uint32_t & out_error);

        /**
        * @brief 位置計數 → 輸出端角度（rad）
        */
        double countsToRad(int32_t cnt) const;

        /**
        * @brief 電機端速度（0.01 Hz）→ 輸出端角速度（rad/s）
        */
        double centiHzToRadPerSec(double speed_centi_hz) const;

    private:
        /**
        * @brief SocketCAN 介面
        */
        SocketCanInterface & socketCan_;

        /**
        * @brief CAN ID
        */
        uint16_t can_id_;

        /**
        * @brief 輸出軸一圈計數 (cpr)
        */
        double counts_per_revolution_;

        /**
        * @brief 減速比（電機端 / 輸出端）；未設定時為 1，速度會是電機端的值
        */
        double reduction_ratio_{1.0};

        /**
        * @brief 輸出軸一圈計數 (cpr)
        */
        double outputCntsPerRevolution() const;

        /**
        * @brief 發送指令並等待回覆
        * @param cmd 指令
        * @param out_data 輸出資料
        * @param out_dlc 輸出資料長度
        * @param timeout_ms 超時時間，單位：毫秒
        * @return true：發送成功，false：發送失敗
        */
        bool sendAndWaitReplyRaw(uint8_t cmd, std::array<uint8_t, 8> & out_data, uint8_t & out_dlc, int timeout_ms);

        /**
        * @brief 在 timeout 內接收，直到收到本馬達 ID 且 accept 回傳 true 的 frame；其他 frame 丟棄
        * @return true：收到符合的 frame，false：超時或接收失敗
        */
        template<typename Accept>
        bool receiveMatching(struct can_frame & frame, int timeout_ms, Accept accept);
};

}  // namespace can_driver

