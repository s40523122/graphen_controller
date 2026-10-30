#include "ti5_hardware/ti5_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "can_driver/can_config_loader.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace ti5_hardware
{

namespace
{
// 馬達的電流與錯誤碼沒有對應的標準介面，以自訂名稱導出（出現在 /dynamic_joint_states）；
// 名稱需與 ti5_170a_graphen.ros2_control.xacro 的 <state_interface> 一致
constexpr char kCurrentInterface[] = "current";
constexpr char kErrorCodeInterface[] = "error_code";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

std::vector<hardware_interface::StateInterface> Ti5Hardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string & name = joint_names_[i];
        interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
        interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
        interfaces.emplace_back(name, kCurrentInterface, &hw_filter_currents_[i]);
        interfaces.emplace_back(name, kErrorCodeInterface, &hw_error_codes_[i]);
        if (control_mode_ == ControlMode::kEffort) {
            interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
        }
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> Ti5Hardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (control_mode_ == ControlMode::kEffort) {
            interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[i]);
        } else {
            interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
        }
    }
    return interfaces;
}

hardware_interface::CallbackReturn Ti5Hardware::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // 關節限位從 robot_description 讀取（original_xml 是 resource manager 收到的完整 URDF）
    if (info.original_xml.empty()) {
        RCLCPP_ERROR(
            rclcpp::get_logger("Ti5Hardware"),
            "HardwareInfo::original_xml is empty; cannot read joint limits from robot_description.");
        return CallbackReturn::ERROR;
    }

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

    // 讀取指令最大變化速度（rad/s），預設 0.0（不限速）
    max_command_velocity_ = 0.0;
    auto it_vel = info.hardware_parameters.find("max_command_velocity");
    if (it_vel != info.hardware_parameters.end() && !it_vel->second.empty()) {
        try {
            const double vel = std::stod(it_vel->second);
            if (!std::isfinite(vel) || vel < 0.0) {
                RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                             "Invalid max_command_velocity=%f (must be >= 0).", vel);
                return CallbackReturn::ERROR;
            }
            max_command_velocity_ = vel;
        } catch (const std::exception & e) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Failed to parse max_command_velocity='%s' (%s).",
                         it_vel->second.c_str(), e.what());
            return CallbackReturn::ERROR;
        }
    }
    if (max_command_velocity_ > 0.0) {
        RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"),
                    "Command velocity limited to %.3f rad/s.", max_command_velocity_);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("Ti5Hardware"),
                    "max_command_velocity is not set, commands are sent without rate limiting.");
    }

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

    // 指令模式由 URDF 決定（hardware:=real 為 position、real_pd 為 effort），所有 joint 必須一致
    std::string mode_interface;
    for (const hardware_interface::ComponentInfo & joint : mapped_joints) {
        if (joint.command_interfaces.size() != 1 ||
            (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION &&
             joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Joint '%s' must have exactly one 'position' or 'effort' command interface.",
                         joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        const std::string & name = joint.command_interfaces[0].name;
        if (mode_interface.empty()) {
            mode_interface = name;
        } else if (name != mode_interface) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Joint '%s' uses '%s' command interface, but other joints use '%s'; "
                         "mixed modes are not supported.",
                         joint.name.c_str(), name.c_str(), mode_interface.c_str());
            return CallbackReturn::ERROR;
        }
    }
    control_mode_ = mode_interface == hardware_interface::HW_IF_EFFORT ?
        ControlMode::kEffort : ControlMode::kPosition;
    RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"), "Command mode: %s.", mode_interface.c_str());

    if (control_mode_ == ControlMode::kEffort) {
        auto it_missed = info.hardware_parameters.find("max_missed_cycles");
        if (it_missed == info.hardware_parameters.end() || it_missed->second.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Missing or empty hardware parameter 'max_missed_cycles' (required in effort mode).");
            return CallbackReturn::ERROR;
        }
        try {
            max_missed_cycles_ = std::stoi(it_missed->second);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Failed to parse max_missed_cycles='%s' (%s).", it_missed->second.c_str(), e.what());
            return CallbackReturn::ERROR;
        }
        if (max_missed_cycles_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Invalid max_missed_cycles=%d (must be >= 0).", max_missed_cycles_);
            return CallbackReturn::ERROR;
        }
    }

    try {
        joint_safety_control_.setLimitsFromJoints(
            mapped_joints, info.original_xml, rclcpp::get_logger("Ti5Hardware"));
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"), "Failed to set joint limits: %s", e.what());
        return CallbackReturn::ERROR;
    }

    // effort 模式以 URDF <limit effort> 作為硬體端的最後一道力矩上限
    if (control_mode_ == ControlMode::kEffort) {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            const double limit = joint_safety_control_.effortLimit(i);
            if (!std::isfinite(limit) || limit <= 0.0) {
                RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                             "Joint '%s' needs a positive URDF <limit effort> in effort mode, got %f.",
                             joint_names_[i].c_str(), limit);
                return CallbackReturn::ERROR;
            }
        }
    }

    // 初始化 joint 狀態、速度、命令
    hw_states_.resize(joint_names_.size(), 0.0);
    hw_velocities_.resize(joint_names_.size(), 0.0);
    hw_commands_.resize(joint_names_.size(), 0.0);
    hw_output_commands_.resize(joint_names_.size(), 0.0);
    hw_filter_currents_.resize(joint_names_.size(), 0.0);
    hw_error_codes_.resize(joint_names_.size(), 0.0);
    hw_effort_commands_.assign(joint_names_.size(), kNaN);
    hw_efforts_.assign(joint_names_.size(), 0.0);
    torque_constants_.assign(joint_names_.size(), 0.0);
    reduction_ratios_.assign(joint_names_.size(), 1.0);
    is_clamping_.assign(joint_names_.size(), false);

    RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"),
                "Initialized %zu joints (from URDF, mapped to CAN channels).", joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        try {
            double lim_min, lim_max;
            joint_safety_control_.getLimit(i, lim_min, lim_max);
            RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"),
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

    // 減速比與 KT 從馬達讀取（不在設定檔重複一份）：速度回饋是電機端，需要減速比換算；
    // effort 模式另外需要 KT 把力矩換成電流。只讀取，不寫入任何馬達參數
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const auto motor = joint_builders_[i]->getMotor(joint_to_motor_index_[i]);
        int32_t ratio = 0;
        if (!motor->readReductionRatio(ratio, hw_read_timeout_ms_) || ratio <= 0) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Read reduction ratio failed or invalid: joint='%s' ratio=%d.",
                         joint_names_[i].c_str(), ratio);
            is_can_ready_ = false;
            return CallbackReturn::ERROR;
        }
        reduction_ratios_[i] = static_cast<double>(ratio);
        motor->setReductionRatio(reduction_ratios_[i]);

        if (control_mode_ == ControlMode::kEffort) {
            float kt = 0.0f;
            if (!motor->readTorqueConstant(kt, hw_read_timeout_ms_) || !std::isfinite(kt) || kt <= 0.0f) {
                RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                             "Read torque constant failed or invalid: joint='%s' KT=%f.",
                             joint_names_[i].c_str(), static_cast<double>(kt));
                is_can_ready_ = false;
                return CallbackReturn::ERROR;
            }
            // TODO(verify-on-hw): 假設 KT 為電機端 Nm/A，輸出端力矩 = 電流 × KT × 減速比（協議 7.5.2 的公式）
            torque_constants_[i] = static_cast<double>(kt);
            RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"),
                        "Joint '%s' reduction ratio %d, KT %.5f Nm/A (%.3f Nm/A at output).",
                        joint_names_[i].c_str(), ratio, torque_constants_[i],
                        torque_constants_[i] * reduction_ratios_[i]);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"),
                        "Joint '%s' reduction ratio %d.", joint_names_[i].c_str(), ratio);
        }
    }

    // effort 模式依通道分組，read() 時同一通道的所有馬達一起送出查詢
    channel_groups_.clear();
    if (control_mode_ == ControlMode::kEffort) {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            auto it = std::find_if(channel_groups_.begin(), channel_groups_.end(),
                                   [&](const ChannelGroup & g) {return g.builder == joint_builders_[i];});
            if (it == channel_groups_.end()) {
                ChannelGroup group;
                group.builder = joint_builders_[i];
                group.joint_of_motor.assign(static_cast<size_t>(group.builder->dof()), kNoJoint);
                channel_groups_.push_back(std::move(group));
                it = std::prev(channel_groups_.end());
            }
            it->joint_of_motor[joint_to_motor_index_[i]] = i;
        }
        channel_feedback_.assign(channel_groups_.size(), can_driver::ChannelFeedback{});
        for (size_t k = 0; k < channel_groups_.size(); ++k) {
            channel_feedback_[k].motors.reserve(channel_groups_[k].joint_of_motor.size());
        }
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

    // Lifecycle reset: 清除 is_command_invalid_，允許再次輸出命令。
    is_command_invalid_ = false;
    is_clamping_.assign(joint_names_.size(), false);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::shared_ptr<can_driver::CanDriverBuilder> & b = joint_builders_[i];
        if (!b) {
            RCLCPP_ERROR(
                rclcpp::get_logger("Ti5Hardware"),
                "Missing builder for channel '%s' (joint '%s') during activate.",
                joint_to_channel_[i].c_str(), joint_names_[i].c_str());
            return CallbackReturn::ERROR;
        }
        // 以 encoder 的實際位置作為輸出起點，讓走向初始姿態的過程受 max_command_velocity 限速
        // TODO(verify-on-hw): 原本 activate 時的 encoder 讀取被註解掉，需確認此時讀取位置是否可靠
        int32_t cnt = 0;
        double actual = 0.0;
        if (!b->readPosition(joint_to_motor_index_[i], cnt, actual, hw_read_timeout_ms_)) {
            RCLCPP_ERROR(
                rclcpp::get_logger("Ti5Hardware"),
                "Activate read failed: joint='%s' channel='%s' motor_index=%zu timeout_ms=%d",
                joint_names_[i].c_str(),
                joint_to_channel_[i].c_str(),
                joint_to_motor_index_[i],
                hw_read_timeout_ms_);
            return CallbackReturn::ERROR;
        }

        hw_states_[i] = actual;
        hw_velocities_[i] = 0.0;

        if (control_mode_ == ControlMode::kEffort) {
            // effort 模式：先以位置模式保持在目前位置；初始姿態由 controller 負責，
            // 所有 joint 收到第一個力矩命令後才切換到電流模式（見 writeEffortMode）
            hw_output_commands_[i] = actual;
            hw_effort_commands_[i] = kNaN;
            if (!b->moveToPositionRad(joint_to_motor_index_[i], actual)) {
                RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                             "Activate position hold failed: joint='%s'.", joint_names_[i].c_str());
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO(rclcpp::get_logger("joint_init_angle"),
                        "Joint '%s' actual %f, holding until effort commands arrive",
                        joint_names_[i].c_str(), actual);
        } else {
            // 目標：initial_positions_file 有設定時使用該值，否則保持目前位置
            double target = actual;
            const YAML::Node init_section = initial_positions_["initial_positions"];
            const YAML::Node init_value = init_section ? init_section[joint_names_[i]] : YAML::Node();
            if (init_value) {
                try {
                    target = init_value.as<double>();
                } catch (const YAML::Exception & e) {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("Ti5Hardware"),
                        "Invalid initial position for joint '%s': %s",
                        joint_names_[i].c_str(), e.what());
                    return CallbackReturn::ERROR;
                }
            }
            RCLCPP_INFO(
                rclcpp::get_logger("joint_init_angle"),
                "Joint '%s' actual %f, initial target %f",
                joint_names_[i].c_str(), actual, target);

            hw_output_commands_[i] = actual;
            hw_commands_[i] = target;
        }

        // 嘗試清除錯誤
        if (hw_error_codes_[i] != 0){
            b->clearError(joint_to_motor_index_[i]);
            RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"), "Joint '%s' try to clear error.", joint_names_[i].c_str());
        }
    }
    consecutive_read_fail_cycles_ = 0;
    torque_active_ = false;
    position_hold_fallback_ = false;
    missed_cycles_ = 0;
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ti5Hardware::on_deactivate(const rclcpp_lifecycle::State &) {
    // effort 模式停用時不留下最後的電流命令（controller 已停止更新），改為位置保持
    if (control_mode_ == ControlMode::kEffort && torque_active_ && !position_hold_fallback_) {
        enterPositionHold("deactivate");
    }
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ti5Hardware::on_cleanup(const rclcpp_lifecycle::State &) {
    is_can_ready_ = false;
    is_command_invalid_ = false;
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
    if (is_command_invalid_) {
        return hardware_interface::return_type::ERROR;
    }
    return control_mode_ == ControlMode::kEffort ? readEffortMode() : readPositionMode();
}

hardware_interface::return_type Ti5Hardware::readPositionMode() {
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

hardware_interface::return_type Ti5Hardware::write(const rclcpp::Time &, const rclcpp::Duration & period) {
    if (!is_can_ready_) {
        return hardware_interface::return_type::ERROR;
    }
    if (is_command_invalid_) {
        return hardware_interface::return_type::ERROR;
    }
    return control_mode_ == ControlMode::kEffort ? writeEffortMode() : writePositionMode(period);
}

hardware_interface::return_type Ti5Hardware::writePositionMode(const rclcpp::Duration & period) {

    // NaN / inf 無法 clamp，代表上層有問題：停止所有輸出，直到 lifecycle reset
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (!std::isfinite(hw_commands_[i])) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Command is not finite: joint='%s' cmd=%f. Stop.",
                         joint_names_[i].c_str(), hw_commands_[i]);
            is_command_invalid_ = true;
            return hardware_interface::return_type::ERROR;
        }
    }

    // 每週期最多移動 max_command_velocity * dt；period 異常（<= 0）時本週期不移動
    const double dt = std::max(period.seconds(), 0.0);
    const double max_step = max_command_velocity_ * dt;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::shared_ptr<can_driver::CanDriverBuilder> & b = joint_builders_[i];
        if (b) {
            // 超出限位時夾在限位上繼續輸出；hw_commands_ 屬於 controller，不直接修改
            const double target = joint_safety_control_.clamp(i, hw_commands_[i]);
            const bool clamping = target != hw_commands_[i];
            if (clamping != is_clamping_[i]) {
                // 只在進入 / 離開 clamp 時各印一次，避免每週期洗 log
                is_clamping_[i] = clamping;
                if (clamping) {
                    double lim_min = 0.0, lim_max = 0.0;
                    joint_safety_control_.getLimit(i, lim_min, lim_max);
                    RCLCPP_WARN(rclcpp::get_logger("Ti5Hardware"),
                                "Command out of limits, clamped: joint='%s' cmd=%.6f rad limit=[%.6f, %.6f].",
                                joint_names_[i].c_str(), hw_commands_[i], lim_min, lim_max);
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"),
                                "Command back within limits: joint='%s'.", joint_names_[i].c_str());
                }
            }

            if (max_command_velocity_ > 0.0) {
                hw_output_commands_[i] += std::clamp(
                    target - hw_output_commands_[i], -max_step, max_step);
            } else {
                hw_output_commands_[i] = target;
            }
            b->moveToPositionRad(joint_to_motor_index_[i], hw_output_commands_[i]);
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

hardware_interface::return_type Ti5Hardware::readEffortMode() {
    if (position_hold_fallback_) {
        return hardware_interface::return_type::ERROR;
    }

    // 所有通道先送出查詢再收回覆，各通道的匯流排傳輸互相重疊；共用同一個 deadline
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(hw_read_timeout_ms_);
    for (ChannelGroup & group : channel_groups_) {
        group.builder->drainReceived();
        group.builder->requestFeedbackAll(true);
    }

    bool complete = true;
    for (size_t k = 0; k < channel_groups_.size(); ++k) {
        const ChannelGroup & group = channel_groups_[k];
        can_driver::ChannelFeedback & fb = channel_feedback_[k];
        group.builder->collectFeedbackAll(deadline, fb);

        for (size_t m = 0; m < group.joint_of_motor.size(); ++m) {
            const size_t i = group.joint_of_motor[m];
            if (i == kNoJoint) {
                continue;
            }
            if (!fb.motors[m]) {
                complete = false;
                continue;
            }
            const auto motor = group.builder->getMotor(m);
            const can_driver::MotorFeedback & f = *fb.motors[m];
            hw_states_[i] = motor->countsToRad(f.position_cnt);
            hw_velocities_[i] = motor->centiHzToRadPerSec(f.speed_centi_hz);
            hw_filter_currents_[i] = f.current_ma;
            hw_efforts_[i] = f.current_ma * 1e-3 * torque_constants_[i] * reduction_ratios_[i];
        }

        if (fb.error && group.joint_of_motor[fb.error->first] != kNoJoint) {
            const size_t i = group.joint_of_motor[fb.error->first];
            const double code = static_cast<double>(fb.error->second);
            if (code != hw_error_codes_[i] && code != 0.0) {
                RCLCPP_WARN(rclcpp::get_logger("Ti5Hardware"),
                            "Motor error: joint='%s' error_code=0x%08X.",
                            joint_names_[i].c_str(), fb.error->second);
            }
            hw_error_codes_[i] = code;
        }
    }

    if (!complete) {
        ++missed_cycles_;
        if (missed_cycles_ == 1) {
            RCLCPP_WARN(rclcpp::get_logger("Ti5Hardware"),
                        "Feedback missing (timeout_ms=%d), using last state.", hw_read_timeout_ms_);
        }
        if (missed_cycles_ > max_missed_cycles_) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Feedback missing for %d cycles (max_missed_cycles=%d).",
                         missed_cycles_, max_missed_cycles_);
            enterPositionHold("feedback lost");
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }
    if (missed_cycles_ > 0) {
        RCLCPP_WARN(rclcpp::get_logger("Ti5Hardware"),
                    "Feedback recovered after %d missing cycles.", missed_cycles_);
        missed_cycles_ = 0;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ti5Hardware::writeEffortMode() {
    if (position_hold_fallback_) {
        return hardware_interface::return_type::ERROR;
    }

    if (!torque_active_) {
        // 所有 joint 都收到第一個有效力矩前維持位置保持，避免部分關節先無力
        for (const double cmd : hw_effort_commands_) {
            if (!std::isfinite(cmd)) {
                return hardware_interface::return_type::OK;
            }
        }
        torque_active_ = true;
        RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"), "All effort commands received, switching to current mode.");
    }

    // 啟動後出現 NaN / inf 代表上層有問題：退回位置保持，直到 lifecycle reset
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (!std::isfinite(hw_effort_commands_[i])) {
            RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                         "Effort command is not finite: joint='%s' cmd=%f.",
                         joint_names_[i].c_str(), hw_effort_commands_[i]);
            is_command_invalid_ = true;
            enterPositionHold("invalid command");
            return hardware_interface::return_type::ERROR;
        }
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const double limit = joint_safety_control_.effortLimit(i);
        const double effort = std::clamp(hw_effort_commands_[i], -limit, limit);
        const bool clamping = effort != hw_effort_commands_[i];
        if (clamping != is_clamping_[i]) {
            // 只在進入 / 離開 clamp 時各印一次，避免每週期洗 log
            is_clamping_[i] = clamping;
            if (clamping) {
                RCLCPP_WARN(rclcpp::get_logger("Ti5Hardware"),
                            "Effort command out of limit, clamped: joint='%s' cmd=%.3f Nm limit=%.3f Nm.",
                            joint_names_[i].c_str(), hw_effort_commands_[i], limit);
            } else {
                RCLCPP_INFO(rclcpp::get_logger("Ti5Hardware"),
                            "Effort command back within limit: joint='%s'.", joint_names_[i].c_str());
            }
        }

        // 輸出端力矩 → 電機電流（mA）：I = τ / (KT × 減速比)
        const double current_ma = effort / (torque_constants_[i] * reduction_ratios_[i]) * 1e3;
        joint_builders_[i]->getMotor(joint_to_motor_index_[i])->setTargetCurrentMa(
            static_cast<int32_t>(std::lround(current_ma)));
    }
    return hardware_interface::return_type::OK;
}

void Ti5Hardware::enterPositionHold(const char * reason) {
    RCLCPP_ERROR(rclcpp::get_logger("Ti5Hardware"),
                 "Switching all motors to position hold (%s); torque output stops until reactivation.",
                 reason);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        hw_output_commands_[i] = hw_states_[i];
        joint_builders_[i]->moveToPositionRad(joint_to_motor_index_[i], hw_states_[i]);
    }
    torque_active_ = false;
    position_hold_fallback_ = true;
}

}  // namespace ti5_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ti5_hardware::Ti5Hardware, hardware_interface::SystemInterface)

