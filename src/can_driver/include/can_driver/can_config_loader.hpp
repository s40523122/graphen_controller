#pragma once

#include <string>
#include <unordered_map>
#include <utility>

namespace can_driver
{

using JointChannelMap = std::unordered_map<std::string, std::pair<std::string, size_t>>;

/**
 * @brief 從 can_driver_config.json 建立 joint name -> (channel, motor_index)
 * @param config_path config 檔案路徑
 * @return 有 motors[].name 的馬達才會列入；失敗或無資料回傳空 map
 */
JointChannelMap loadJointMapFromConfig(const std::string & config_path);

}  // namespace can_driver
