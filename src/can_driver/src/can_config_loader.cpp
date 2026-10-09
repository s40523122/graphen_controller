#include "can_driver/can_config_loader.hpp"

#include <fstream>

#include "nlohmann/json.hpp"

namespace can_driver
{

JointChannelMap loadJointMapFromConfig(const std::string & config_path)
{
    JointChannelMap m;
    std::ifstream ifs(config_path);
    if (!ifs) {
        return m;
    }
    nlohmann::json j;
    try {
        ifs >> j;
    } catch (...) {
        return m;
    }
    if (!j.contains("CAN") || !j["CAN"].is_object()) {
        return m;
    }
    for (auto it = j["CAN"].begin(); it != j["CAN"].end(); ++it) {
        const std::string & channel_name = it.key();
        const auto & ch = it.value();
        if (!ch.contains("motors") || !ch["motors"].is_array()) {
            continue;
        }
        size_t idx = 0;
        for (const auto & mot : ch["motors"]) {
            if (mot.contains("name") && mot["name"].is_string()) {
                std::string name = mot["name"].get<std::string>();
                if (!name.empty()) {
                    m[std::move(name)] = {channel_name, idx};
                }
            }
            ++idx;
        }
    }
    return m;
}

}  // namespace can_driver
