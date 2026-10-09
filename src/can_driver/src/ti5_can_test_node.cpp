#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "can_driver/can_manager.hpp"

using namespace std::chrono_literals;

class CanDriverTestNode : public rclcpp::Node
{
    public:
        CanDriverTestNode()
        : Node("can_driver_test_node")
        {
            std::string config_path = this->declare_parameter<std::string>("config_path", "");
            if (config_path.empty()) {
                config_path =
                ament_index_cpp::get_package_share_directory("can_driver") + "/config/can_driver_config.json";
            }
            const std::string channel_name = "left_leg";

            RCLCPP_INFO(
            get_logger(), "CAN driver config: '%s', channel: '%s'", config_path.c_str(), channel_name.c_str());

            manager_ = std::make_shared<can_driver::CanManager>(config_path);
            if (!manager_->initialize()) {
                RCLCPP_FATAL(get_logger(), "CanManager initialize failed");
                throw std::runtime_error("CanManager initialize failed");
            }
            builder_ = manager_->getBuilderPtr(channel_name);
            if (!builder_) {
                RCLCPP_FATAL(get_logger(), "Missing builder for channel '%s'", channel_name.c_str());
                throw std::runtime_error("Missing builder for requested channel");
            }

            timer_ = this->create_wall_timer(1s, std::bind(&CanDriverTestNode::on_timer, this));
        }

    private:
        void on_timer()
        {
            timer_->cancel();

            int32_t pos_cnt = 0;
            double pos_rad = 0.0;
            if (!builder_->readPosition(pos_cnt, pos_rad, 500)) {
                RCLCPP_WARN(get_logger(), "Failed to read position");
            } else {
                const double pos_deg = pos_rad * 180.0 / M_PI;
                RCLCPP_INFO(get_logger(), "Current position: %d cnt (%.3f rad, %.3f deg)", pos_cnt, pos_rad, pos_deg);
            }

            const double step_rad = 5.0 * M_PI / 180.0;  // 5 deg
            RCLCPP_INFO(get_logger(), "Moving +5 deg (%.3f rad)...", step_rad);
            if (!builder_->moveRelativeRad(step_rad, 500)) {
                RCLCPP_ERROR(get_logger(), "Failed to command +5 deg");
            }

            rclcpp::sleep_for(2s);

            RCLCPP_INFO(get_logger(), "Moving -5 deg (%.3f rad)...", -step_rad);
            if (!builder_->moveRelativeRad(-step_rad, 500)) {
                RCLCPP_ERROR(get_logger(), "Failed to command -5 deg");
            }

            uint32_t err = 0;
            if (builder_->readError(err, 200)) {
                RCLCPP_INFO(get_logger(), "Motor error code: 0x%08X", err);
            }

            RCLCPP_INFO(get_logger(), "Test sequence done, shutting down node.");
            rclcpp::shutdown();
        }

        std::shared_ptr<can_driver::CanManager> manager_;
        std::shared_ptr<can_driver::CanDriverBuilder> builder_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<CanDriverTestNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::fprintf(stderr, "Exception in can_driver_test_node: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    return 0;
}

