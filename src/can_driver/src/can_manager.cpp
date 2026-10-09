#include "can_driver/can_manager.hpp"

#include <fstream>
#include <iostream>

#include "nlohmann/json.hpp"

namespace can_driver
{

CanManager::CanManager(const std::string & config_file)
    : config_file_(config_file) {
}

bool CanManager::initialize() {
    // 先釋放舊資源（避免持有舊 socket）
    shutdown();

    std::ifstream ifs(config_file_);
    if (!ifs) {
        std::cerr << "[can_manager] Failed to open config file: '" << config_file_ << "'\n";
        return false;
    }

    nlohmann::json config;
    try {
        ifs >> config;
    } catch (const nlohmann::json::exception & e) {
        std::cerr << "[can_manager] Failed to parse config file '" << config_file_ << "': " << e.what() << "\n";
        return false;
    }

    if (!config.contains("CAN") || !config["CAN"].is_object() || config["CAN"].empty()) {
        std::cerr << "[can_manager] Config missing param 'CAN' or it has no channels.\n";
        return false;
    }

    // 通道清單完全由 config 決定，不在程式中指定 group 名稱
    for (const auto & item : config["CAN"].items()) {
        const std::string & channel_name = item.key();
        auto builder = std::make_shared<CanDriverBuilder>(config_file_, channel_name);
        if (!builder->initialize()) {
            std::cerr << "[can_manager] Failed to initialize channel '" << channel_name << "'.\n";
            return false;
        }
        builders_.emplace(channel_name, std::move(builder));
    }

    return true;
}

void CanManager::shutdown() noexcept
{
    builders_.clear();
}

std::shared_ptr<CanDriverBuilder> CanManager::getBuilderPtr(const std::string & channel_name) const {
    const auto it = builders_.find(channel_name);
    return it != builders_.end() ? it->second : nullptr;
}

std::vector<std::string> CanManager::channelNames() const {
    std::vector<std::string> names;
    names.reserve(builders_.size());
    for (const auto & entry : builders_) {
        names.push_back(entry.first);
    }
    return names;
}

}  // namespace can_driver
