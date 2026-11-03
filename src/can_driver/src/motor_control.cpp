#include "can_driver/motor_control.hpp"

#include <chrono>
#include <cmath>
#include <cstring>

namespace can_driver
{

MotorControl::MotorControl(SocketCanInterface & socketCan, uint16_t can_id, double counts_per_revolution)
    : socketCan_(socketCan),
      can_id_(can_id),
      counts_per_revolution_(counts_per_revolution) {
}

bool MotorControl::sendOneByte(OneByteCommand cmd) {
    uint8_t data = static_cast<uint8_t>(cmd);
    return socketCan_.sendFrame(can_id_, &data, 1);
}

bool MotorControl::sendOneByteWithReply(OneByteCommand cmd, std::array<uint8_t, 8> & out_data,
                                        uint8_t & out_dlc, int timeout_ms) {
    return sendAndWaitReplyRaw(static_cast<uint8_t>(cmd), out_data, out_dlc, timeout_ms);
}

bool MotorControl::sendFiveByte(FiveByteCommand cmd, int32_t param) {
    uint8_t buf[5];
    buf[0] = static_cast<uint8_t>(cmd);
    // 32-bit little-endian
    buf[1] = static_cast<uint8_t>(param & 0xFF);
    buf[2] = static_cast<uint8_t>((param >> 8) & 0xFF);
    buf[3] = static_cast<uint8_t>((param >> 16) & 0xFF);
    buf[4] = static_cast<uint8_t>((param >> 24) & 0xFF);

    return socketCan_.sendFrame(can_id_, buf, 5);
}

bool MotorControl::readPositionCnt(int32_t & out_cnt, int timeout_ms) {
    std::array<uint8_t, 8> data{};
    uint8_t dlc = 0;
    if (!sendOneByteWithReply(OneByteCommand::GetPosition, data, dlc, timeout_ms)) {
        return false;
    }

    if (dlc != 5 || data[0] != static_cast<uint8_t>(OneByteCommand::GetPosition)) {
        return false;
    }

    int32_t value = static_cast<int32_t>(
    (static_cast<uint32_t>(data[1])      ) |
    (static_cast<uint32_t>(data[2]) << 8 ) |
    (static_cast<uint32_t>(data[3]) << 16) |
    (static_cast<uint32_t>(data[4]) << 24));

    out_cnt = value;
    return true;
}

bool MotorControl::readPositionRad(double & out_rad, int timeout_ms) {
    int32_t cnt = 0;
    if (!readPositionCnt(cnt, timeout_ms)) {
        return false;
    }
    out_rad = countsToRad(cnt);
    return true;
}

double MotorControl::countsToRad(int32_t cnt) const {
    // cnt -> revolutions -> radians (2*pi per revolution)
    return static_cast<double>(cnt) / outputCntsPerRevolution() * 2.0 * M_PI;
}

double MotorControl::centiHzToRadPerSec(double speed_centi_hz) const {
    // 協議的速度一律是電機端 0.01 Hz（0.01 rps），不分單編 / 雙編，輸出端要再除以減速比
    return speed_centi_hz * 0.01 * 2.0 * M_PI / reduction_ratio_;
}

bool MotorControl::moveToPositionCnt(int32_t target_cnt) {
    return sendFiveByte(FiveByteCommand::SetTargetPosition, target_cnt);
}

bool MotorControl::moveToPositionRad(double target_rad) {
    double rev_cnts = outputCntsPerRevolution();
    // rad -> revolutions -> cnt
    double cnt_f = target_rad / (2.0 * M_PI) * rev_cnts;
    int32_t target_cnt = static_cast<int32_t>(std::round(cnt_f));
    return moveToPositionCnt(target_cnt);
}

bool MotorControl::moveRelativeCnt(int32_t delta_cnt, int timeout_ms_for_readback) {
    int32_t current = 0;
    if (!readPositionCnt(current, timeout_ms_for_readback)) {
        return false;
    }
    int32_t target = current + delta_cnt;
    return moveToPositionCnt(target);
}

bool MotorControl::moveRelativeRad(double delta_rad, int timeout_ms_for_readback) {
    double rev_cnts = outputCntsPerRevolution();
    double delta_cnt_f = delta_rad / (2.0 * M_PI) * rev_cnts;
    int32_t delta_cnt = static_cast<int32_t>(std::round(delta_cnt_f));
    return moveRelativeCnt(delta_cnt, timeout_ms_for_readback);
}

bool MotorControl::readSpeedRad(double & out_rad_per_sec, int timeout_ms) {
    std::array<uint8_t, 8> data{};
    uint8_t dlc = 0;
    if (!sendOneByteWithReply(OneByteCommand::GetSpeed, data, dlc, timeout_ms)) {
        return false;
    }

    if (dlc != 5 || data[0] != static_cast<uint8_t>(OneByteCommand::GetSpeed)) {
        return false;
    }

    // 協議 7.1：0x06 回傳電機端速度，單位 0.01 Hz
    int32_t value = static_cast<int32_t>(
        (static_cast<uint32_t>(data[1])      ) |
        (static_cast<uint32_t>(data[2]) << 8 ) |
        (static_cast<uint32_t>(data[3]) << 16) |
        (static_cast<uint32_t>(data[4]) << 24));

    out_rad_per_sec = centiHzToRadPerSec(static_cast<double>(value));
    return true;
}

bool MotorControl::readFilterCurrent(double & out_ma, int timeout_ms) {
    std::array<uint8_t, 8> data{};
    uint8_t dlc = 0;
    if (!sendOneByteWithReply(OneByteCommand::GetFilteredCurrent, data, dlc, timeout_ms)) {
        return false;
    }

    if (dlc != 5 || data[0] != static_cast<uint8_t>(OneByteCommand::GetFilteredCurrent)) {
        return false;
    }

    int32_t value = static_cast<int32_t>(
    (static_cast<uint32_t>(data[1])      ) |
    (static_cast<uint32_t>(data[2]) << 8 ) |
    (static_cast<uint32_t>(data[3]) << 16) |
    (static_cast<uint32_t>(data[4]) << 24));

    out_ma = static_cast<double>(value);
    return true;
}

bool MotorControl::readErrorCode(uint32_t & out_error, int timeout_ms) {
    std::array<uint8_t, 8> data{};
    uint8_t dlc = 0;
    if (!sendOneByteWithReply(OneByteCommand::GetErrorStatus, data, dlc, timeout_ms)) {
        return false;
    }

    if (dlc != 5 || data[0] != static_cast<uint8_t>(OneByteCommand::GetErrorStatus)) {
        return false;
    }

    uint32_t value =
    (static_cast<uint32_t>(data[1])      ) |
    (static_cast<uint32_t>(data[2]) << 8 ) |
    (static_cast<uint32_t>(data[3]) << 16) |
    (static_cast<uint32_t>(data[4]) << 24);

    out_error = value;
    return true;
}

bool MotorControl::clearError() {
    return sendOneByte(OneByteCommand::ClearError);
}

bool MotorControl::readSixByteParam(SixByteCommand cmd, std::array<uint8_t, 4> & out_raw, int timeout_ms) {
    const uint8_t code = static_cast<uint8_t>(cmd);
    const uint8_t buf[6] = {code, kSixByteRead, 0, 0, 0, 0};
    if (!socketCan_.sendFrame(can_id_, buf, 6)) {
        return false;
    }

    struct can_frame frame;
    const bool ok = receiveMatching(frame, timeout_ms, [code](const struct can_frame & f) {
        return f.can_dlc == 6 && f.data[0] == code && f.data[1] == kSixByteRead;
    });
    if (!ok) {
        return false;
    }
    std::memcpy(out_raw.data(), &frame.data[2], out_raw.size());
    return true;
}

bool MotorControl::readReductionRatio(int32_t & out_ratio, int timeout_ms) {
    std::array<uint8_t, 4> raw{};
    if (!readSixByteParam(SixByteCommand::ReductionRatio, raw, timeout_ms)) {
        return false;
    }
    out_ratio = static_cast<int32_t>(
        (static_cast<uint32_t>(raw[0])      ) |
        (static_cast<uint32_t>(raw[1]) << 8 ) |
        (static_cast<uint32_t>(raw[2]) << 16) |
        (static_cast<uint32_t>(raw[3]) << 24));
    return true;
}

bool MotorControl::readTorqueConstant(float & out_kt, int timeout_ms) {
    std::array<uint8_t, 4> raw{};
    if (!readSixByteParam(SixByteCommand::TorqueConstant, raw, timeout_ms)) {
        return false;
    }
    // TODO(verify-on-hw): 協議寫 32 位浮點數、低位在前，假設為 IEEE 754 little-endian
    static_assert(sizeof(float) == 4, "float must be 32-bit");
    std::memcpy(&out_kt, raw.data(), sizeof(out_kt));
    return true;
}

bool MotorControl::setTargetCurrentMa(int32_t current_ma) {
    return sendFiveByte(FiveByteCommand::SetTargetCurrent, current_ma);
}

bool MotorControl::setTargetCurrentMaWithFeedback(int32_t current_ma) {
    return sendFiveByte(FiveByteCommand::SetCurrentWithFeedback, current_ma);
}

bool MotorControl::requestFeedback() {
    return sendOneByte(OneByteCommand::GetFeedback);
}

bool MotorControl::requestErrorCode() {
    return sendOneByte(OneByteCommand::GetErrorStatus);
}

bool MotorControl::parseFeedback(const struct can_frame & frame, MotorFeedback & out) {
    // 8 位元組回覆沒有指令碼，只能以長度辨識（其他回覆都是 5 或 6 位元組）
    if (frame.can_dlc != 8) {
        return false;
    }
    const uint8_t * d = frame.data;
    out.current_ma = static_cast<int16_t>(
        static_cast<uint16_t>(d[0]) | (static_cast<uint16_t>(d[1]) << 8));
    out.speed_centi_hz = static_cast<int16_t>(
        static_cast<uint16_t>(d[2]) | (static_cast<uint16_t>(d[3]) << 8));
    out.position_cnt = static_cast<int32_t>(
        (static_cast<uint32_t>(d[4])      ) |
        (static_cast<uint32_t>(d[5]) << 8 ) |
        (static_cast<uint32_t>(d[6]) << 16) |
        (static_cast<uint32_t>(d[7]) << 24));
    return true;
}

bool MotorControl::parseErrorCode(const struct can_frame & frame, uint32_t & out_error) {
    if (frame.can_dlc != 5 || frame.data[0] != static_cast<uint8_t>(OneByteCommand::GetErrorStatus)) {
        return false;
    }
    const uint8_t * d = frame.data;
    out_error =
        (static_cast<uint32_t>(d[1])      ) |
        (static_cast<uint32_t>(d[2]) << 8 ) |
        (static_cast<uint32_t>(d[3]) << 16) |
        (static_cast<uint32_t>(d[4]) << 24);
    return true;
}

template<typename Accept>
bool MotorControl::receiveMatching(struct can_frame & frame, int timeout_ms, Accept accept) {
    using Clock = std::chrono::steady_clock;
    const Clock::time_point deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
        const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - Clock::now());
        if (remaining.count() <= 0 || !socketCan_.receiveFrame(frame, remaining)) {
            return false;
        }
        if (frame.can_id == can_id_ && accept(frame)) {
            return true;
        }
    }
}

double MotorControl::outputCntsPerRevolution() const {
    return counts_per_revolution_;
}

bool MotorControl::sendAndWaitReplyRaw(uint8_t cmd, std::array<uint8_t, 8> & out_data,
                                       uint8_t & out_dlc, int timeout_ms) {
    // 送出 1-byte 指令
    if (!socketCan_.sendFrame(can_id_, &cmd, 1)) {
        return false;
    }

    struct can_frame frame;
    const int single_wait = timeout_ms;
    if (!socketCan_.receiveFrame(frame, single_wait)) {
        return false;
    }

    if (frame.can_id != can_id_) {
        return false;
    }

    out_dlc = frame.can_dlc;
    std::memset(out_data.data(), 0, out_data.size());
    std::memcpy(out_data.data(), frame.data, frame.can_dlc);
    return true;
}

}  // namespace can_driver

