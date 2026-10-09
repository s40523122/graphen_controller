#include "can_driver/can_driver_builder.hpp"

#include <cerrno>
#include <fstream>
#include <iostream>

#include "nlohmann/json.hpp"

namespace can_driver
{

CanDriverBuilder::CanDriverBuilder(const std::string & config_path, const std::string & channel_name)
    : config_file_(config_path),
      channel_enable_(false),
      channel_name_(channel_name),
      bitrate_(1000000),
      txqueuelen_(1000),
      can_id_(0),
      dof_(0),
      encoder_dual_outer_(262144.0),
      encoder_single_inner_(65536.0) {
}

CanDriverBuilder::~CanDriverBuilder() {
}

bool CanDriverBuilder::initialize() {
    std::ifstream js(config_file_);
    if (!js) {
        std::cerr << "[can_driver] Failed to open js file: '" << config_file_
                  << "' (errno=" << errno << ")\n";
        return false;
    }

    nlohmann::json canParamJson;
    js >> canParamJson;

    // 讀取 Encoder 預設值
    if (!canParamJson.contains("Encoder")) {
        std::cerr << "[can_driver] Config missing param 'Encoder'.\n";
        return false;
    }
    const auto & encoderJson = canParamJson["Encoder"];

    if (!encoderJson.contains("dual_outer")) {
        std::cerr << "[can_driver] Config Encoder missing param 'dual_outer'.\n";
        return false;
    }
    encoderJson.at("dual_outer").get_to(encoder_dual_outer_);

    if (!encoderJson.contains("single_inner")) {
        std::cerr << "[can_driver] Config Encoder missing param 'single_inner'.\n";
        return false;
    }
    encoderJson.at("single_inner").get_to(encoder_single_inner_);

    // 讀取 CAN 通道設定
    if (!canParamJson.contains("CAN")) {
        std::cerr << "[can_driver] Config missing param 'CAN'.\n";
        return false;
    }
    const auto & canChannelJson = canParamJson["CAN"];

    if (!canChannelJson.contains(channel_name_)) {
        std::cerr << "[can_driver] Config missing channel name : " << channel_name_ << ".\n";
        return false;
    }
    const auto & channelJson = canChannelJson[channel_name_];

    if (!channelJson.contains("enable")) {
        std::cerr << "[can_driver] Config channel [" << channel_name_ << "] missing param 'enable'.\n";
        return false;
    }
    channelJson.at("enable").get_to(channel_enable_);


    if (!channelJson.contains("channel")) {
        std::cerr << "[can_driver] Config channel [" << channel_name_ << "] missing param 'channel'.\n";
        return false;
    }
    channelJson.at("channel").get_to(channel_);

    if (!channelJson.contains("bitrate")) {
        std::cerr << "[can_driver] Config channel [" << channel_name_ << "] missing param 'bitrate'.\n";
        return false;
    }
    channelJson.at("bitrate").get_to(bitrate_);

    if (!channelJson.contains("txqueuelen")) {
        std::cerr << "[can_driver] Config channel [" << channel_name_ << "] missing param 'txqueuelen'.\n";
        return false;
    }
    channelJson.at("txqueuelen").get_to(txqueuelen_);

    if (!channelJson.contains("can_id")) {
        std::cerr << "[can_driver] Config channel [" << channel_name_ << "] missing param 'can_id'.\n";
        return false;
    }
    channelJson.at("can_id").get_to(can_id_);

    if (!channelJson.contains("dof")) {
        std::cerr << "[can_driver] Config channel [" << channel_name_ << "] missing param 'dof'.\n";
        return false;
    }
    channelJson.at("dof").get_to(dof_);

    // 如果 channel 未啟用，則直接返回 true
    if (!channel_enable_) {
        std::cerr << "[can_driver] Channel [" << channel_name_ << "] is disabled in config.\n";
        return true;
    }

    // 建立 SocketCAN 介面
    socketCan_ = std::make_shared<SocketCanInterface>(channel_);
    if (!socketCan_->open()) {
        std::cerr << "[can_driver] Failed to open CAN channel '" << channel_name_ << "'\n";
        return false;
    }

    // 檢查並建立 motors：
    // 每顆馬達輸出軸一圈計數 (cpr) 由 encoder_type + reduction_ratio 決定：
    // DualOuter = Encoder.dual_outer
    // SingleInner = Encoder.single_inner * reduction_ratio
    if (!channelJson.contains("motors") || !channelJson["motors"].is_array()) {
        std::cerr << "[can_driver] Config channel [" << channel_name_
                  << "] missing param 'motors' or it is not an array.\n";
        return false;
    }
    const auto & motorsJson = channelJson["motors"];
    if (static_cast<int>(motorsJson.size()) != dof_) {
        std::cerr << "[can_driver] Config channel [" << channel_name_
                  << "] 'motors' size (" << motorsJson.size()
                  << ") does not match dof (" << dof_ << ").\n";
        return false;
    }

    motors_.clear();
    motors_.reserve(static_cast<size_t>(dof_));
    for (int i = 0; i < dof_; ++i) {
        double cpr = encoder_dual_outer_;
        const auto & mj = motorsJson[i];

        // 必須有 encoder_type
        if (!mj.contains("encoder_type") || !mj["encoder_type"].is_string()) {
            std::cerr << "[can_driver] Config channel [" << channel_name_
                      << "] motor index " << i << " missing param 'encoder_type'.\n";
            return false;
        }
        std::string enc_type = mj["encoder_type"].get<std::string>();

        if (enc_type == "SingleInner") {
            // SingleInner 必須提供 reduction_ratio
            if (!mj.contains("reduction_ratio") || !mj["reduction_ratio"].is_number_integer()) {
                std::cerr << "[can_driver] Config channel [" << channel_name_
                          << "] motor index " << i
                          << " missing integer param 'reduction_ratio' for SingleInner encoder.\n";
                return false;
            }
            const int reduction = mj["reduction_ratio"].get<int>();
            cpr = encoder_single_inner_ * static_cast<double>(reduction);
        } else {
            // 其他型別一律視為 DualOuter
            cpr = encoder_dual_outer_;
        }

        motors_.push_back(std::make_shared<MotorControl>(
            *socketCan_, static_cast<uint16_t>(can_id_ + i), cpr));
    }

    std::cerr << "[can_driver] Initialized channel '" << channel_name_ << "' with " << dof_ << " motors (can_id "
              << can_id_ << ".." << (can_id_ + dof_ - 1) << ").\n";
    return true;
}

std::shared_ptr<MotorControl> CanDriverBuilder::getMotor(size_t index) const {
    if (index >= motors_.size()) {
        return nullptr;
    }
    return motors_[index];
}

bool CanDriverBuilder::readPosition(size_t motor_index, int32_t & out_cnt, double & out_rad, int timeout_ms) {
    if (motor_index >= motors_.size()) {
        return false;
    }
    auto & m = motors_[motor_index];
    if (!m) return false;
    if (!m->readPositionCnt(out_cnt, timeout_ms)) return false;
    return m->readPositionRad(out_rad, timeout_ms);
}

bool CanDriverBuilder::readPosition(int32_t & out_cnt, double & out_rad, int timeout_ms) {
    return readPosition(0, out_cnt, out_rad, timeout_ms);
}

bool CanDriverBuilder::moveRelativeRad(size_t motor_index, double delta_rad, int timeout_ms_for_readback) {
    if (motor_index >= motors_.size()) return false;
    auto & m = motors_[motor_index];
    return m ? m->moveRelativeRad(delta_rad, timeout_ms_for_readback) : false;
}

bool CanDriverBuilder::moveRelativeRad(double delta_rad, int timeout_ms_for_readback) {
    return moveRelativeRad(0, delta_rad, timeout_ms_for_readback);
}

bool CanDriverBuilder::moveToPositionRad(size_t motor_index, double target_rad) {
    if (motor_index >= motors_.size()) return false;
    auto & m = motors_[motor_index];
    return m ? m->moveToPositionRad(target_rad) : false;
}

bool CanDriverBuilder::moveToPositionRad(double target_rad) {
    return moveToPositionRad(0, target_rad);
}

bool CanDriverBuilder::readFilterCurrent(size_t motor_index, double & out_ma, int timeout_ms) {
    if (motor_index >= motors_.size()) {
        return false;
    }
    auto & m = motors_[motor_index];
    if (!m) return false;
    if (!m->readFilterCurrent(out_ma, timeout_ms)) return false;
    return m->readFilterCurrent(out_ma, timeout_ms);
}

bool CanDriverBuilder::readError(size_t motor_index, uint32_t & out_error, int timeout_ms) {
    if (motor_index >= motors_.size()) return false;
    auto & m = motors_[motor_index];
    return m ? m->readErrorCode(out_error, timeout_ms) : false;
}

bool CanDriverBuilder::readError(uint32_t & out_error, int timeout_ms) {
    return readError(0, out_error, timeout_ms);
}

bool CanDriverBuilder::clearError(size_t motor_index) {
    if (motor_index >= motors_.size()) return false;
    auto & m = motors_[motor_index];
    return m ? m->clearError() : false;
}

bool CanDriverBuilder::readSpeed(size_t motor_index, double & out_rad_per_sec, int timeout_ms) {
    if (motor_index >= motors_.size()) {
        return false;
    }
    auto & m = motors_[motor_index];
    return m ? m->readSpeedRad(out_rad_per_sec, timeout_ms) : false;
}

bool CanDriverBuilder::readSpeed(double & out_rad_per_sec, int timeout_ms) {
    return readSpeed(0, out_rad_per_sec, timeout_ms);
}

}  // namespace can_driver

