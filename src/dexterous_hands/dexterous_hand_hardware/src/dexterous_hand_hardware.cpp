#include "dexterous_hand_hardware/dexterous_hand_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>

namespace dexterous_hand_hardware
{

hardware_interface::CallbackReturn DexterousHandHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;

  

  size_t joint_size = info_.joints.size();

  hw_positions_.resize(joint_size, 0.0);
  hw_velocities_.resize(joint_size, 0.0);
  hw_efforts_.resize(joint_size, 0.0);
  hw_commands_.resize(joint_size, 0.0);

  hw_currents_.resize(joint_size, 0.0);
  hw_errors_.resize(joint_size, 0.0);
  hw_states_.resize(joint_size, 0.0);
  hw_temperatures_.resize(joint_size, 0.0);

  tactile_normal_.resize(8, 0.0);
  tactile_tangent_.resize(8, 0.0);
  tactile_direction_.resize(8, 0.0);
  
  last_sent_raw_.resize(13, -9999);

  joint_conversions_ = {
    {  960, 1650,  1.3089969,        0.0 },  // [0]  pinky_mcp
    {  970, 1650,  1.3089969,        0.0 },  // [1]  ring_mcp
    {  950, 1650,  1.3089969,        0.0 },  // [2]  middle_mcp
    {  150, -150,    0.26118,   -0.26118 },  // [3]  middle_yaw
    {  980, 1650,  1.3089969,        0.0 },  // [4]  index_mcp
    {  150, -150,    0.26118,   -0.26118 },  // [5]  index_yaw
    { 1180, 1900,  1.4835299,        0.0 },  // [6]  pinky_pip
    { 1150, 1900,  1.4835299,        0.0 },  // [7]  ring_pip
    { 1150, 1900,  1.4835299,        0.0 },  // [8]  middle_pip
    { 1150, 1900,  1.4835299,        0.0 },  // [9]  index_pip
    {  650, 1750,  1.9198622,        0.0 },  // [10] thumb_yaw
    { 1300, 1560,  0.6108652,        0.0 },  // [11] thumb_mcp
    { 1500, 2040,  1.041888,         0.0 },  // [12] thumb_pip
  };

  // Setting : ros2_control.xacro
  std::string port    = info_.hardware_parameters.at("serial_port");
  hand_id_            = std::stoi(info_.hardware_parameters.at("hand_id"));
  int baudrate        = std::stoi(info_.hardware_parameters.at("baudrate"));

  if (!comm_.connect(port, baudrate)) {
    RCLCPP_ERROR(rclcpp::get_logger("DexterousHandHardware"),
      "無法連線到串口 %s", port.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  comm_.set_speed(hand_id_, 800);  // 預設速度

  RCLCPP_INFO(rclcpp::get_logger("DexterousHandHardware"),
    "初始化完成，共 %zu 個關節", info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DexterousHandHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, 
      hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, 
      hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, 
      hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);

    state_interfaces.emplace_back(info_.joints[i].name, 
      "current", &hw_currents_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, 
      "error", &hw_errors_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, 
      "status", &hw_states_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, 
      "temperature", &hw_temperatures_[i]);
  }

  for (size_t i = 0; i < info_.sensors.size(); i++) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.sensors[i].name, "tactile_normal", &tactile_normal_[i]));
          
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.sensors[i].name, "tactile_tangent", &tactile_tangent_[i]));
          
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.sensors[i].name, "tactile_direction", &tactile_direction_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DexterousHandHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, 
      hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DexterousHandHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_commands_[i] = hw_positions_[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("DexterousHandHardware"), "硬體介面已啟動");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DexterousHandHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DexterousHandHardware"), "硬體介面已停止");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DexterousHandHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // ==========================================
  // 區塊 A：關節核心資料 (地址 1136 ~ 1218，共 83 個暫存器)
  // ==========================================
  static const int JOINT_MAP[13] = {
    10, 11, 12, 5, 4, 9, 3, 2, 8, 1, 7, 0, 6
  };
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  std::vector<int16_t> block_a;
  if (comm_.read_registers(hand_id_, 1136, 83, block_a)) {
    for (size_t i = 0; i < 13; i++) {
      int hw_idx = JOINT_MAP[i];

      int16_t raw_pos = block_a[hw_idx];
      if (i == 1){
        raw_pos += 40;
      }

      hw_positions_[i] = raw_to_rad(raw_pos, hw_idx);
      hw_efforts_[i]      = block_a[14 + hw_idx] * 9.81 / 1000.0; // g 轉 N
      hw_currents_[i]     = block_a[28 + hw_idx] * 0.001;         // mA 轉 A
      hw_errors_[i]       = static_cast<double>(block_a[42 + hw_idx]);
      hw_states_[i]       = static_cast<double>(block_a[56 + hw_idx]);
      hw_temperatures_[i] = static_cast<double>(block_a[70 + hw_idx]);

      // hw_positions_[i]    = raw_to_rad(block_a[i], i);
      // hw_efforts_[i]      = block_a[14 + i] * 9.81 / 1000.0; // g 轉 N
      // hw_currents_[i]     = block_a[28 + i] * 0.001;         // mA 轉 A
      // hw_errors_[i]       = static_cast<double>(block_a[42 + i]);
      // hw_states_[i]       = static_cast<double>(block_a[56 + i]);
      // hw_temperatures_[i] = static_cast<double>(block_a[70 + i]);
    }
  }

  // ==========================================
  // 區塊 B：觸覺感測器資料 (地址 3000 ~ 3033，共 34 個暫存器)
  // ==========================================
  std::vector<uint16_t> block_b;
  if (comm_.read_registers_uint(hand_id_, 3000, 34, block_b)) {
    // 1. 處理 5 根手指 (每根佔 5 個暫存器)
    for (size_t i = 0; i < 5; i++) {
      tactile_normal_[i]    = block_b[i * 5 + 0] * 0.01; // 0.01N 轉 N
      tactile_tangent_[i]   = block_b[i * 5 + 1] * 0.01;
      tactile_direction_[i] = static_cast<double>(block_b[i * 5 + 2]);
    }
    // 2. 處理 3 個掌心區域 (從偏移量 25 開始，每個佔 3 個暫存器)
    for (size_t i = 0; i < 3; i++) {
      size_t offset = 25 + (i * 3);
      tactile_normal_[5 + i]    = block_b[offset + 0] * 0.01;
      tactile_tangent_[5 + i]   = block_b[offset + 1] * 0.01;
      tactile_direction_[5 + i] = static_cast<double>(block_b[offset + 2]);
    }
  }

  hw_velocities_.assign(hw_velocities_.size(), 0.0);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DexterousHandHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  static constexpr double MAX_JUMP = 100.0;
  static constexpr int16_t RAW_DEADBAND = 10;

  static const int JOINT_MAP[13] = {
    10, 11, 12, 5, 4, 9, 3, 2, 8, 1, 7, 0, 6
  };
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  if (hw_commands_.size() != 13) {
    RCLCPP_ERROR(rclcpp::get_logger("DexterousHandHardware"),
      "hw_commands_ 長度異常: %zu", hw_commands_.size());
    return hardware_interface::return_type::ERROR;
  }

  bool first_time = (last_sent_raw_.size() != 13);
  bool need_send = false;
  std::vector<int16_t> raw_angles(13);

  for (size_t i = 0; i < hw_commands_.size(); i++) {
    int hw_idx = JOINT_MAP[i]; 
    double cmd = hw_commands_[i];

    if (std::abs(cmd - hw_positions_[i]) > MAX_JUMP) {
      RCLCPP_WARN(rclcpp::get_logger("DexterousHandHardware"),
        "關節 %zu 指令跳變過大 (cmd=%.3f, pos=%.3f)，已攔截",
        i, cmd, hw_positions_[i]);
      cmd = hw_positions_[i];
    }

    int16_t new_raw = rad_to_raw(cmd, hw_idx);
    if (first_time || std::abs(new_raw - last_sent_raw_[hw_idx]) > RAW_DEADBAND) {
      raw_angles[hw_idx] = new_raw;
      need_send = true;
    }
    else {
      raw_angles[hw_idx] = last_sent_raw_[hw_idx];
    }
  }

  if (need_send) {
    RCLCPP_INFO(rclcpp::get_logger("DexterousHandHardware"),
      "write: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
      raw_angles[0],  raw_angles[1],  raw_angles[2],  raw_angles[3],
      raw_angles[4],  raw_angles[5],  raw_angles[6],  raw_angles[7],
      raw_angles[8],  raw_angles[9],  raw_angles[10], raw_angles[11],
      raw_angles[12]);

    bool ok = comm_.write_angles(hand_id_, raw_angles);
    if (ok) {
      last_sent_raw_ = raw_angles;  // ✅ 更新記錄
    } else {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("DexterousHandHardware"),
        steady_clock, 2000,
        "write_angles 回傳失敗");
    }
  }
  // for (size_t i = 0; i < hw_commands_.size(); i++) {
  //   raw_angles[i] = rad_to_raw(hw_commands_[i], i);
  // }
  // comm_.write_angles(hand_id_, raw_angles);

  return hardware_interface::return_type::OK;
}

int16_t DexterousHandHardware::rad_to_raw(double rad, size_t joint_idx)
{
  if (joint_idx >= joint_conversions_.size()) return 0;
  const auto & c = joint_conversions_[joint_idx];

  double ratio = (rad - c.rad_min) / (c.rad_max - c.rad_min);
  ratio = std::clamp(ratio, 0.0, 1.0);
  return static_cast<int16_t>(c.raw_min + ratio * (c.raw_max - c.raw_min));
}

double DexterousHandHardware::raw_to_rad(int16_t raw, size_t joint_idx)
{
  if (joint_idx >= joint_conversions_.size()) return 0.0;
  const auto & c = joint_conversions_[joint_idx];

  double ratio = static_cast<double>(raw - c.raw_min) /
                 static_cast<double>(c.raw_max - c.raw_min);
  ratio = std::clamp(ratio, 0.0, 1.0);
  return c.rad_min + ratio * (c.rad_max - c.rad_min);
}

}  // namespace dexterous_hand_hardware

PLUGINLIB_EXPORT_CLASS(
  dexterous_hand_hardware::DexterousHandHardware,
  hardware_interface::SystemInterface)
