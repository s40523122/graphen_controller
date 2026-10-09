#include "ti5_hardware/ti5_hardware.hpp"

#include <optional>
#include <string>

#include "can_driver/can_config_loader.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace ti5_hardware
{

std::vector<hardware_interface::StateInterface> Ti5Hardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string & name = joint_names_[i];
        interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
        //interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
        interfaces.emplace_back(name, hardware_interface::HW_IF_ACCELERATION, &hw_filter_currents_[i]);
        interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_error_codes_[i]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> Ti5Hardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
    }
    return interfaces;
}

hardware_interface::CallbackReturn Ti5Hardware::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // 讀取 URDF 檔案路徑（用於讀取 <joint><limit lower/upper>）
    auto it_urdf = info.hardware_parameters.find("urdf_path");
    if (it_urdf == info.hardware_parameters.end() || it_urdf->second.empty()) {
        RCLCPP_ERROR(
            rclcpp::get_logger("Ti5Hardware"),
            "Missing or empty hardware parameter 'urdf_path'.");
        return CallbackReturn::ERROR;
    }
    urdf_path_ = it_urdf->second;

    // 讀取 CAN 設定檔路徑
    auto it = info.hardware_parameters.find("can_config_path");
    if (it == info.hardware_parameters.end() || it->second.empty()) {
        RCLCPP_ERROR(
            rclcpp::get_logger("Ti5Hardware"),
            "Missing or empty hardware parameter 'can_config_path'.");
        return CallbackReturn::ERROR;
    }
    can_config_path_ = it->second;

    // 讀取 CAN read timeout（ms）
    auto it_to = info.hardware_parameters.find("hw_read_timeout_ms");
    if (it_to == info.hardware_parameters.end() || it_to->second.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                     "Missing or empty hardware parameter 'hw_read_timeout_ms'.");
        return CallbackReturn::ERROR;
    }
    try {
        const int v = std::stoi(it_to->second);
        if (v <= 0) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Invalid hw_read_timeout_ms=%d (must be > 0).", v);
            return CallbackReturn::ERROR;
        }
        hw_read_timeout_ms_ = v;
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                     "Failed to parse hw_read_timeout_ms='%s' (%s).",
                     it_to->second.c_str(), e.what());
        return CallbackReturn::ERROR;
    }

    // 讀取關節限位判定容差（弧度），預設 0.0
    auto it_eps = info.hardware_parameters.find("limit_epsilon_rad");
    if (it_eps != info.hardware_parameters.end() && !it_eps->second.empty()) {
        try {
            const double eps = std::stod(it_eps->second);
            if (eps < 0.0) {
                RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                             "Invalid limit_epsilon_rad=%.9f (must be >= 0).", eps);
                return CallbackReturn::ERROR;
            }
            limit_epsilon_rad_ = eps;
        } catch (const std::exception & e) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Failed to parse limit_epsilon_rad='%s' (%s).",
                         it_eps->second.c_str(), e.what());
            return CallbackReturn::ERROR;
        }
    } else {
        limit_epsilon_rad_ = 0.0;
    }
    joint_safety_control_.setLimitEpsilon(limit_epsilon_rad_);

    // 讀取出使關節角度設定檔路徑
    auto info_init = info.hardware_parameters.at("initial_positions_file");
    try
    {
        initial_positions_ = YAML::LoadFile(info_init);
    }
    catch (const YAML::Exception & e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("Ti5Hardware"),
            "Failed to load initial_positions_file: %s",
            e.what());

        return CallbackReturn::ERROR;
    }
    

    // 初始化 CAN 準備狀態（真正的 CAN 初始化在 on_configure）
    is_can_ready_ = false;

    // 從 CAN config 建立 joint name -> (channel, motor_index)（依 motors[].name）
    const can_driver::JointChannelMap joint_map = can_driver::loadJointMapFromConfig(can_config_path_);
    if (joint_map.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                     "Failed to load joint mapping from config '%s' (missing or no motors[].name).",
                     can_config_path_.c_str());
        return CallbackReturn::ERROR;
    }

    // 清空 joint 相關指標
    joint_names_.clear();
    joint_to_channel_.clear();
    joint_to_motor_index_.clear();

    std::vector<hardware_interface::ComponentInfo> mapped_joints;

    // 從 URDF 讀取所有的 joint 資訊，依 config 對應建立 channel / motor_index，並讀取關節限位
    for (const hardware_interface::ComponentInfo & joint : info.joints) {
        auto jit = joint_map.find(joint.name);
        if (jit == joint_map.end()) {
            RCLCPP_WARN(rclcpp::get_logger("Ti5Hardware"), "Joint '%s' has no CAN mapping, skipping.", joint.name.c_str());
            continue;
        }
        const std::string & ch = jit->second.first;
        const size_t motor_idx = jit->second.second;
        joint_names_.push_back(joint.name);
        joint_to_channel_.push_back(ch);
        joint_to_motor_index_.push_back(motor_idx);
        mapped_joints.push_back(joint);
    }

    if (joint_names_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"), "No joints mapped to CAN motors.");
        return CallbackReturn::ERROR;
    }

    try {
        joint_safety_control_.setLimitsFromJoints(
            mapped_joints, std::optional<std::string>(urdf_path_), rclcpp::get_logger("Ti5Hardware"));
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"), "Failed to set joint limits: %s", e.what());
        return CallbackReturn::ERROR;
    }

    // 初始化 joint 狀態、速度、命令
    hw_states_.resize(joint_names_.size(), 0.0);
    hw_velocities_.resize(joint_names_.size(), 0.0);
    hw_commands_.resize(joint_names_.size(), 0.0);
    hw_filter_currents_.resize(joint_names_.size(), 0.0);
    hw_error_codes_.resize(joint_names_.size(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"),
                "Initialized %zu joints (from URDF, mapped to CAN channels).", joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        try {
            double lim_min, lim_max;
            joint_safety_control_.getLimit(i, lim_min, lim_max);
            RCLCPP_DEBUG(rclcpp::get_logger("Ti5Hardware"),
                         "Joint '%s' position limits [%.4f, %.4f] rad.",
                         joint_names_[i].c_str(), lim_min, lim_max);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(
                rclcpp::get_logger("Ti5Hardware"),
                "Joint limit index mismatch at i=%zu (joint='%s'): %s",
                i, joint_names_[i].c_str(), e.what());
            return CallbackReturn::ERROR;
        }
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ti5Hardware::on_configure(const rclcpp_lifecycle::State &) {
    if (!can_manager_) {
        can_manager_ = std::make_shared<can_driver::CanManager>(can_config_path_);
    }

    if (!can_manager_->initialize()) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                     "CanManager initialize failed (config='%s')",
                     can_config_path_.c_str());

        is_can_ready_ = false;
        return CallbackReturn::ERROR;
    }

    // 建立 joint_index -> builder 快取，並做一致性檢查
    joint_builders_.clear();
    joint_builders_.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string & ch = joint_to_channel_[i];
        const size_t motor_idx = joint_to_motor_index_[i];
        auto b = can_manager_->getBuilderPtr(ch);
        if (!b) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"), "Missing builder for channel '%s' (joint '%s').",
                        ch.c_str(), joint_names_[i].c_str());
            is_can_ready_ = false;
            return CallbackReturn::ERROR;
        }

        if (motor_idx >= static_cast<size_t>(b->dof()) || !b->getMotor(motor_idx)) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Motor index %zu out of range for channel '%s' (dof=%d) (joint '%s').",
                         motor_idx, ch.c_str(), b->dof(), joint_names_[i].c_str());
            is_can_ready_ = false;
            return CallbackReturn::ERROR;
        }
        joint_builders_[i] = b;
    }

    is_can_ready_ = true;
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ti5Hardware::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"), "Ti5Hardware on activate...");
    if (!is_can_ready_) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"), "Cannot activate: CAN not ready (configure failed).");
        return CallbackReturn::ERROR;
    }

    // Lifecycle reset: 清除 is_over_limit_，允許再次輸出命令。
    is_over_limit_ = false;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::shared_ptr<can_driver::CanDriverBuilder> & b = joint_builders_[i];
        if (!b) {
            RCLCPP_ERROR(
                rclcpp::get_logger("Ti5Hardware"),
                "Missing builder for channel '%s' (joint '%s') during activate.",
                joint_to_channel_[i].c_str(), joint_names_[i].c_str());
            return CallbackReturn::ERROR;
        }
        int32_t cnt = 0;
        double rad = 0.0;

        try
        {
            rad = initial_positions_["initial_positions"][joint_names_[i]].as<double>();
            RCLCPP_INFO(
                rclcpp::get_logger("joint_init_angle"),
                "Joint '%s' in init is '%f'",
                joint_names_[i].c_str(), rad
            );

            // if (!b->readPosition(joint_to_motor_index_[i], cnt, rad, hw_read_timeout_ms_)) {
            //     RCLCPP_ERROR(
            //         rclcpp::get_logger("Ti5Hardware"),
            //         "Activate read failed: joint='%s' channel='%s' motor_index=%zu timeout_ms=%d",
            //         joint_names_[i].c_str(),
            //         joint_to_channel_[i].c_str(),
            //         joint_to_motor_index_[i],
            //         hw_read_timeout_ms_);
            //     return CallbackReturn::ERROR;
            // }

            // RCLCPP_INFO(
            //     rclcpp::get_logger("joint_get_angle"),
            //     "Joint '%s' by get is '%f'",
            //     joint_names_[i].c_str(), rad
            // );

        }
        catch (const std::exception & e)
        {
            if (!b->readPosition(joint_to_motor_index_[i], cnt, rad, hw_read_timeout_ms_)) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("Ti5Hardware"),
                    "Activate read failed: joint='%s' channel='%s' motor_index=%zu timeout_ms=%d",
                    joint_names_[i].c_str(),
                    joint_to_channel_[i].c_str(),
                    joint_to_motor_index_[i],
                    hw_read_timeout_ms_);
                return CallbackReturn::ERROR;
            }
        }

        
        hw_states_[i] = rad;
        hw_commands_[i] = hw_states_[i];

        // 嘗試清除錯誤
        if (hw_error_codes_[i] != 0){
            b->clearError(joint_to_motor_index_[i]);
            RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"), "Joint '%s' try to clear error.", joint_names_[i].c_str());
        }
    }
    consecutive_read_fail_cycles_ = 0;
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ti5Hardware::on_deactivate(const rclcpp_lifecycle::State &) {
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ti5Hardware::on_cleanup(const rclcpp_lifecycle::State &) {
    is_can_ready_ = false;
    is_over_limit_ = false;
    consecutive_read_fail_cycles_ = 0;
    joint_builders_.clear();
    if (can_manager_) {
        can_manager_->shutdown();
    }
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ti5Hardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration &) {
    if (!is_can_ready_) {
        return hardware_interface::return_type::ERROR;
    }
    if (is_over_limit_) {
        return hardware_interface::return_type::ERROR;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::shared_ptr<can_driver::CanDriverBuilder> & b = joint_builders_[i];
        if (!b) {
            ++consecutive_read_fail_cycles_;
            if (consecutive_read_fail_cycles_ == 1) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("Ti5Hardware"),
                    "Read failed: missing builder cache for joint='%s' channel='%s'.",
                    joint_names_[i].c_str(),
                    joint_to_channel_[i].c_str());
            }
            return hardware_interface::return_type::ERROR;
        }
        int32_t cnt = 0;
        double rad = 0.0;
        if (!b->readPosition(joint_to_motor_index_[i], cnt, rad, hw_read_timeout_ms_)) {
            ++consecutive_read_fail_cycles_;
            if (consecutive_read_fail_cycles_ == 1) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("Ti5Hardware"),
                    "Read position failed: joint='%s' channel='%s' motor_index=%zu timeout_ms=%d",
                    joint_names_[i].c_str(),
                    joint_to_channel_[i].c_str(),
                    joint_to_motor_index_[i],
                    hw_read_timeout_ms_);
            }
            return hardware_interface::return_type::ERROR;
        }
        hw_states_[i] = rad;

        double vel_rad_per_sec = 0.0;
        if (!b->readSpeed(joint_to_motor_index_[i], vel_rad_per_sec, hw_read_timeout_ms_)) {
            ++consecutive_read_fail_cycles_;
            if (consecutive_read_fail_cycles_ == 1) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("Ti5Hardware"),
                    "Read velocity failed: joint='%s' channel='%s' motor_index=%zu timeout_ms=%d",
                    joint_names_[i].c_str(),
                    joint_to_channel_[i].c_str(),
                    joint_to_motor_index_[i],
                    hw_read_timeout_ms_);
            }
            return hardware_interface::return_type::ERROR;
        }
        hw_velocities_[i] = vel_rad_per_sec;

        double f_current = 0.0;
        if (!b->readFilterCurrent(joint_to_motor_index_[i], f_current, hw_read_timeout_ms_)) {
            ++consecutive_read_fail_cycles_;
            if (consecutive_read_fail_cycles_ == 1) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("Ti5Hardware"),
                    "Read filter current failed: joint='%s' channel='%s' motor_index=%zu timeout_ms=%d",
                    joint_names_[i].c_str(),
                    joint_to_channel_[i].c_str(),
                    joint_to_motor_index_[i],
                    hw_read_timeout_ms_);
            }
            return hardware_interface::return_type::ERROR;
        }
        hw_filter_currents_[i] = f_current;

        uint32_t out_error_code = 0.0;
        if (!b->readError(joint_to_motor_index_[i], out_error_code, hw_read_timeout_ms_)) {
            ++consecutive_read_fail_cycles_;
            if (consecutive_read_fail_cycles_ == 1) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("Ti5Hardware"),
                    "Read error code failed: joint='%s' channel='%s' motor_index=%zu timeout_ms=%d",
                    joint_names_[i].c_str(),
                    joint_to_channel_[i].c_str(),
                    joint_to_motor_index_[i],
                    hw_read_timeout_ms_);
            }
            return hardware_interface::return_type::ERROR;
        }
        hw_error_codes_[i] = static_cast<double>(out_error_code);
    }

    if (consecutive_read_fail_cycles_ > 0) {
        RCLCPP_WARN(
            rclcpp::get_logger("Ti5Hardware"),
            "read 曾連續失敗 %d 次，已恢復。",
            consecutive_read_fail_cycles_);
        consecutive_read_fail_cycles_ = 0;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ti5Hardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
    if (!is_can_ready_) {
        return hardware_interface::return_type::ERROR;
    }
    if (is_over_limit_) {
        return hardware_interface::return_type::ERROR;
    }

    try {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            if (!joint_safety_control_.isCommandWithinLimits(i, hw_commands_[i])) {
                double lim_min = 0.0, lim_max = 0.0;
                joint_safety_control_.getLimit(i, lim_min, lim_max);
                RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                             "Command out of limits: joint='%s' cmd=%.6f rad limit=[%.6f, %.6f]. Stop.",
                             joint_names_[i].c_str(), hw_commands_[i], lim_min, lim_max);
                is_over_limit_ = true;
                return hardware_interface::return_type::ERROR;
            }
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"), "Joint limit check failed: %s", e.what());
        is_over_limit_ = true;
        return hardware_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::shared_ptr<can_driver::CanDriverBuilder> & b = joint_builders_[i];
        if (b) {
            b->moveToPositionRad(joint_to_motor_index_[i], hw_commands_[i]);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Write failed: missing builder cache for joint='%s' channel='%s'.",
                         joint_names_[i].c_str(),
                         joint_to_channel_[i].c_str());
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

}  // namespace ti5_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ti5_hardware::Ti5Hardware, hardware_interface::SystemInterface)

