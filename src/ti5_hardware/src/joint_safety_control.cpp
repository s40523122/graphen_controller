#include "ti5_hardware/joint_safety_control.hpp"

#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "urdf/model.h"

namespace ti5_hardware
{

void JointSafetyControl::setLimits(const std::vector<LimitPair> & limits) {
    limits_ = limits;
}

void JointSafetyControl::setLimitsFromJoints(const std::vector<hardware_interface::ComponentInfo> & joints_in_order,
                                             const std::string & urdf_xml,
                                             const rclcpp::Logger & logger) {
    // 限位一律以 robot_description 為準，與 controller / BVH retarget 使用同一份 URDF
    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_xml)) {
        throw std::runtime_error("Failed to parse URDF (robot_description) for joint limits");
    }

    std::vector<LimitPair> position_limits;
    position_limits.reserve(joints_in_order.size());
    std::vector<double> effort_limits;
    effort_limits.reserve(joints_in_order.size());

    // 解析失敗時回傳 std::nullopt（並印 WARN），讓下一個來源補上
    const auto parse = [&logger](const std::string & joint_name, const char * key, const std::string & text)
        -> std::optional<double> {
        if (text.empty()) {
            return std::nullopt;
        }
        try {
            return std::stod(text);
        } catch (const std::exception &) {
            RCLCPP_WARN(logger, "Joint '%s' position %s '%s' parse failed, ignored.",
                        joint_name.c_str(), key, text.c_str());
            return std::nullopt;
        }
    };

    for (const auto & joint : joints_in_order) {
        // 上下限各自依優先順序取第一個有效來源，所以只設定其中一邊時另一邊仍由 URDF 補上
        std::optional<double> min_pos;
        std::optional<double> max_pos;

        // 1. ros2_control 的 command_interface(position) min/max
        for (const hardware_interface::InterfaceInfo & iface : joint.command_interfaces) {
            if (iface.name == hardware_interface::HW_IF_POSITION) {
                min_pos = parse(joint.name, "min", iface.min);
                max_pos = parse(joint.name, "max", iface.max);
                break;
            }
        }

        // 2. ros2_control 的 joint.parameters["min"/"max"]
        if (!min_pos) {
            const auto it = joint.parameters.find("min");
            if (it != joint.parameters.end()) {
                min_pos = parse(joint.name, "min", it->second);
            }
        }
        if (!max_pos) {
            const auto it = joint.parameters.find("max");
            if (it != joint.parameters.end()) {
                max_pos = parse(joint.name, "max", it->second);
            }
        }

        // 3. URDF 的 <limit lower/upper>
        if (!min_pos || !max_pos) {
            const urdf::JointConstSharedPtr uj = urdf_model.getJoint(joint.name);
            if (!uj) {
                RCLCPP_ERROR(logger, "URDF is missing joint '%s' (required for limits).", joint.name.c_str());
                throw std::runtime_error("URDF missing joint '" + joint.name + "'");
            }
            if (!uj->limits || uj->type == urdf::Joint::CONTINUOUS) {
                RCLCPP_ERROR(
                    logger,
                    "Joint '%s' has no <limit lower/upper>; explicit limits are required.",
                    joint.name.c_str());
                throw std::runtime_error("Joint '" + joint.name + "' has no limits");
            }
            if (!min_pos) {
                min_pos = uj->limits->lower;
            }
            if (!max_pos) {
                max_pos = uj->limits->upper;
            }
        }

        if (*min_pos > *max_pos) {
            std::swap(*min_pos, *max_pos);
            RCLCPP_WARN(logger, "Joint '%s' limits min > max, swapped.", joint.name.c_str());
        }

        position_limits.emplace_back(*min_pos, *max_pos);

        const urdf::JointConstSharedPtr uj = urdf_model.getJoint(joint.name);
        effort_limits.push_back(uj && uj->limits ? uj->limits->effort : 0.0);
    }

    setLimits(position_limits);
    effort_limits_ = std::move(effort_limits);
}

double JointSafetyControl::effortLimit(size_t index) const {
    if (index >= effort_limits_.size()) {
        throw std::out_of_range("JointSafetyControl::effortLimit index out of range");
    }
    return effort_limits_[index];
}

void JointSafetyControl::getLimit(size_t index, double & min_rad_out, double & max_rad_out) const {
    if (index >= limits_.size()) {
        throw std::out_of_range("JointSafetyControl::getLimit index out of range");
    }
    min_rad_out = limits_[index].first;
    max_rad_out = limits_[index].second;
}

double JointSafetyControl::clamp(size_t index, double command_rad) const {
    if (index >= limits_.size()) {
        throw std::out_of_range("JointSafetyControl::clamp index out of range");
    }
    return std::clamp(command_rad, limits_[index].first, limits_[index].second);
}

}  // namespace ti5_hardware
