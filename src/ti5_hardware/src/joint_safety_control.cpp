#include "ti5_hardware/joint_safety_control.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "urdf/model.h"

namespace ti5_hardware
{

void JointSafetyControl::setLimits(const std::vector<LimitPair> & limits) {
    limits_ = limits;
}

void JointSafetyControl::setLimitsFromJoints(const std::vector<hardware_interface::ComponentInfo> & joints_in_order,
                                             const std::optional<std::string> & urdf_path,
                                             const rclcpp::Logger & logger) {
    urdf::Model urdf_model;
    bool urdf_loaded = false;
    if (urdf_path.has_value() && !urdf_path->empty()) {
        if (urdf_model.initFile(*urdf_path)) {
            urdf_loaded = true;
            RCLCPP_INFO(logger, "Loaded URDF from '%s' for joint limits.", urdf_path->c_str());
        } else {
            RCLCPP_WARN(logger, "Failed to load URDF from '%s'; using ros2_control limits/defaults.", urdf_path->c_str());
        }
    }

    std::vector<LimitPair> position_limits;
    position_limits.reserve(joints_in_order.size());

    for (const auto & joint : joints_in_order) {
        double min_pos = -M_PI;
        double max_pos = M_PI;
        bool has_explicit_minmax = false;

        // 優先使用 ros2_control 的 command_interface min/max
        for (const hardware_interface::InterfaceInfo & iface : joint.command_interfaces) {
            if (iface.name == hardware_interface::HW_IF_POSITION) {
                if (!iface.min.empty()) {
                    try {
                        min_pos = std::stod(iface.min);
                        has_explicit_minmax = true;
                    } catch (const std::exception &) {
                        RCLCPP_WARN(
                            logger, "Joint '%s' position min '%s' parse failed, use default.",
                            joint.name.c_str(), iface.min.c_str());
                    }
                }
                if (!iface.max.empty()) {
                    try {
                        max_pos = std::stod(iface.max);
                        has_explicit_minmax = true;
                    } catch (const std::exception &) {
                        RCLCPP_WARN(
                            logger, "Joint '%s' position max '%s' parse failed, use default.",
                            joint.name.c_str(), iface.max.c_str());
                    }
                }
                break;
            }
        }

        // 其次使用 ros2_control 的 joint.parameters["min"/"max"]
        auto pmin = joint.parameters.find("min");
        auto pmax = joint.parameters.find("max");
        if (pmin != joint.parameters.end() && !pmin->second.empty()) {
            try {
                min_pos = std::stod(pmin->second);
                has_explicit_minmax = true;
            } catch (const std::exception &) {}
        }
        if (pmax != joint.parameters.end() && !pmax->second.empty()) {
            try {
                max_pos = std::stod(pmax->second);
                has_explicit_minmax = true;
            } catch (const std::exception &) {}
        }

        // 最終使用 URDF 的 <limit lower/upper>（當 ros2_control 沒提供時）
        if (!has_explicit_minmax && urdf_loaded) {
            const urdf::JointConstSharedPtr uj = urdf_model.getJoint(joint.name);
            if (!uj) {
                RCLCPP_ERROR(logger, "URDF is missing joint '%s' (required for limits).", joint.name.c_str());
                throw std::runtime_error("URDF missing joint '" + joint.name + "'");
            }
            if (!uj->limits) {
                if (uj->type == urdf::Joint::CONTINUOUS) {
                    RCLCPP_ERROR(
                        logger,
                        "Joint '%s' is continuous and has no <limit>; explicit limits are required.",
                        joint.name.c_str());
                    throw std::runtime_error("Joint '" + joint.name + "' has no limits (continuous)");
                }
                RCLCPP_ERROR(
                    logger,
                    "Joint '%s' has no <limit lower/upper>; explicit limits are required.",
                    joint.name.c_str());
                throw std::runtime_error("Joint '" + joint.name + "' has no limits");
            }
            min_pos = uj->limits->lower;
            max_pos = uj->limits->upper;
        }

        if (min_pos > max_pos) {
            std::swap(min_pos, max_pos);
            RCLCPP_WARN(logger, "Joint '%s' limits min > max, swapped.", joint.name.c_str());
        }

        position_limits.emplace_back(min_pos, max_pos);
    }

    setLimits(position_limits);
}

void JointSafetyControl::getLimit(size_t index, double & min_rad_out, double & max_rad_out) const {
    if (index >= limits_.size()) {
        throw std::out_of_range("JointSafetyControl::getLimit index out of range");
    }
    min_rad_out = limits_[index].first;
    max_rad_out = limits_[index].second;
}

bool JointSafetyControl::isCommandWithinLimits(size_t index, double command_rad) const {
    if (index >= limits_.size()) {
        throw std::out_of_range("JointSafetyControl::isCommandWithinLimits index out of range");
    }
    const double min_pos = limits_[index].first;
    const double max_pos = limits_[index].second;
    return (command_rad >= (min_pos - limit_epsilon_rad_) &&
            command_rad <= (max_pos + limit_epsilon_rad_));
}

bool JointSafetyControl::areCommandsWithinLimits(const double * commands, size_t n) const {
    if (!commands) {
        return true;
    }
    if (n != limits_.size()) {
        throw std::out_of_range("JointSafetyControl::areCommandsWithinLimits size mismatch");
    }
    for (size_t i = 0; i < n; ++i) {
        if (!isCommandWithinLimits(i, commands[i])) {
            return false;
        }
    }
    return true;
}

}  // namespace ti5_hardware
