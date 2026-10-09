#include "can_driver/motor_control.hpp"

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

    double rev_cnts = outputCntsPerRevolution();
    // cnt -> revolutions -> radians (2*pi per revolution)
    out_rad = static_cast<double>(cnt) / rev_cnts * 2.0 * M_PI;
    return true;
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

    // 依照 TI5 協議，假設回傳的 32-bit 整數單位為「encoder counts / second」
    int32_t value = static_cast<int32_t>(
        (static_cast<uint32_t>(data[1])      ) |
        (static_cast<uint32_t>(data[2]) << 8 ) |
        (static_cast<uint32_t>(data[3]) << 16) |
        (static_cast<uint32_t>(data[4]) << 24));

    const double rev_cnts = outputCntsPerRevolution();
    // counts/s -> revolutions/s -> rad/s
    out_rad_per_sec = static_cast<double>(value) / rev_cnts * 2.0 * M_PI;
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

