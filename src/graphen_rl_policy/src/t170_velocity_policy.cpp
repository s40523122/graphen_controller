#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "graphen_locomotion/locomotion_algorithm.hpp"
#include "onnxruntime_cxx_api.h"
#include "pluginlib/class_list_macros.hpp"
#include "t170_policy_config.h"

namespace graphen::rl_policy {

using locomotion::Context;
using locomotion::JointCommand;
using locomotion::LogLevel;
using locomotion::Logger;
using locomotion::RobotState;

// FullBodyT70 訓練出的 T170 全身速度追蹤 policy（policy.onnx：obs [1,107] → actions [1,32]）。
//
// obs 組成、相位時鐘、q_home 與 action scale 都與 FullBodyT70/cpp_interface 的 T170Policy 相同
// （常數取自同一份 t170_policy_config.h）；差別是輸入改由 RobotState 提供：
//   gyro / 重力投影 ← base.angular_velocity / base.projected_gravity（BaseStateEstimator）
//   速度指令        ← command.velocity（controller 的 ~/cmd_vel，逾時為 0）
//   關節角度 / 速度 ← measured_positions / measured_velocities
//
// planner 的頻率可以高於 policy：dt 累積滿 policy_dt 才推論一次，其餘週期輸出上一次的目標。
// 機身狀態無效或量測值有 NaN 時不輸出（保持），恢復後繼續。
//
// 參數（<mode>.*）：
//   model_path   policy.onnx 的路徑（必填）；可寫 package://<套件>/<相對路徑>
//   policy_dt    推論間隔（秒，預設為訓練的 kCtrlDt = 0.02）
//   name_map     policy joint 名稱 → controller joint 名稱，"policy:controller" 形式
//                （預設 ["NECK_Y1:NECK_Y"]），沒有列出的名稱視為相同
//   log_period   > 0 時每隔這麼久（秒）log 一次速度指令、重力投影與推論時間（預設 0，不 log）
class T170VelocityPolicy : public locomotion::LocomotionAlgorithm {
public:
  void configure(const Context & context) override {
    log_ = context.log;

    const auto model_path = context.params.get_string("model_path");
    if (!model_path || model_path->empty()) {
      throw std::invalid_argument("'model_path' is required");
    }
    policy_dt_ = context.params.get_double("policy_dt").value_or(t170::kCtrlDt);
    if (!(policy_dt_ > 0.0)) {
      throw std::invalid_argument("'policy_dt' must be greater than zero");
    }
    if (context.rate_hz > 0.0 && policy_dt_ < 1.0 / context.rate_hz - 1e-9) {
      throw std::invalid_argument("'policy_dt' is shorter than the planner period");
    }
    log_period_ = context.params.get_double("log_period").value_or(0.0);

    const auto name_map = context.params.get_string_array("name_map")
      .value_or(std::vector<std::string>{"NECK_Y1:NECK_Y"});
    map_joints(context.joint_names, name_map);
    load_model(resolve_path(*model_path));

    if (log_) {
      log_(LogLevel::kInfo, "T170VelocityPolicy: loaded '" + *model_path + "', policy_dt " +
        std::to_string(policy_dt_) + " s.");
    }
  }

  void on_enter(const RobotState & /*state*/) override {
    clock_ = 0.0;
    last_action_.fill(0.0f);
    // 第一個週期就推論
    since_inference_ = policy_dt_;
    has_target_ = false;
    since_log_ = 0.0;
    holding_ = false;
  }

  void update(double dt, const RobotState & state, JointCommand & command) override {
    since_inference_ += dt;
    if (since_inference_ >= policy_dt_ - 1e-6) {
      if (inputs_valid(state)) {
        infer(state);
        holding_ = false;
      } else if (!holding_) {
        holding_ = true;
        has_target_ = false;
        warn("base state or joint measurements invalid, holding positions");
      }
      // 落後超過一個週期時重新對齊，不連續補跑
      since_inference_ = std::min(since_inference_ - policy_dt_, policy_dt_);
    }
    if (!has_target_) {
      return;
    }
    for (size_t i = 0; i < t170::kNumJoints; ++i) {
      command.positions[index_[i]] = target_[i];
    }

    if (log_period_ > 0.0 && log_) {
      since_log_ += dt;
      if (since_log_ >= log_period_) {
        since_log_ = 0.0;
        const auto & c = state.command.velocity;
        const auto & g = state.base.projected_gravity;
        char buffer[256];
        std::snprintf(buffer, sizeof(buffer),
          "cmd (%.2f, %.2f, %.2f) active %s | gravity (%.3f, %.3f, %.3f) | inference %.3f ms",
          c[0], c[1], c[2], state.command.active ? "true" : "false", g[0], g[1], g[2],
          last_inference_ms_);
        log_(LogLevel::kInfo, buffer);
      }
    }
  }

private:
  void map_joints(
    const std::vector<std::string> & joint_names, const std::vector<std::string> & name_map)
  {
    for (size_t i = 0; i < t170::kNumJoints; ++i) {
      std::string name = t170::kJointNames[i];
      for (const auto & entry : name_map) {
        const auto colon = entry.find(':');
        if (colon == std::string::npos) {
          throw std::invalid_argument("'name_map' entry '" + entry + "' is not 'policy:controller'");
        }
        if (entry.compare(0, colon, name) == 0 && colon == name.size()) {
          name = entry.substr(colon + 1);
          break;
        }
      }
      const auto it = std::find(joint_names.begin(), joint_names.end(), name);
      if (it == joint_names.end()) {
        throw std::invalid_argument(
          "policy joint '" + std::string(t170::kJointNames[i]) + "' (controller name '" + name +
          "') is not a commanded joint");
      }
      index_[i] = static_cast<size_t>(it - joint_names.begin());
    }
  }

  // package://<pkg>/<rel> → <pkg 的 share 目錄>/<rel>；其他字串原樣回傳
  static std::string resolve_path(const std::string & path) {
    const std::string scheme = "package://";
    if (path.compare(0, scheme.size(), scheme) != 0) {
      return path;
    }
    const auto rest = path.substr(scheme.size());
    const auto slash = rest.find('/');
    if (slash == std::string::npos) {
      throw std::invalid_argument("'" + path + "' must be package://<package>/<path>");
    }
    try {
      return ament_index_cpp::get_package_share_directory(rest.substr(0, slash)) +
             rest.substr(slash);
    } catch (const std::exception & e) {
      throw std::invalid_argument("cannot resolve '" + path + "': " + e.what());
    }
  }

  void load_model(const std::string & path) {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "t170_velocity_policy");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);  // 小型 MLP：單執行緒最快且結果固定
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    try {
      session_ = std::make_unique<Ort::Session>(*env_, path.c_str(), options);
    } catch (const Ort::Exception & e) {
      throw std::invalid_argument("cannot load '" + path + "': " + e.what());
    }
    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
    const auto in_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto out_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (in_shape.size() != 2 || in_shape[1] != t170::kObsDim ||
      out_shape.size() != 2 || out_shape[1] != t170::kActDim)
    {
      throw std::invalid_argument(
        "model '" + path + "' does not match t170_policy_config.h (expected obs [1," +
        std::to_string(t170::kObsDim) + "] -> actions [1," + std::to_string(t170::kActDim) + "])");
    }
    memory_info_ = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
  }

  bool inputs_valid(const RobotState & state) const {
    if (!state.base.valid) {
      return false;
    }
    for (size_t i = 0; i < t170::kNumJoints; ++i) {
      if (!std::isfinite(state.measured_positions[index_[i]]) ||
        !std::isfinite(state.measured_velocities[index_[i]]))
      {
        return false;
      }
    }
    return true;
  }

  void infer(const RobotState & state) {
    // 與 T170Policy::step() 相同：指令小於 0.1 時相位輸入為 0（站立），時鐘照常前進
    const auto & cmd = state.command.velocity;
    const double cmd_norm = std::sqrt(cmd[0] * cmd[0] + cmd[1] * cmd[1] + cmd[2] * cmd[2]);
    const double phi = std::fmod(clock_, static_cast<double>(t170::kPhasePeriod)) /
      t170::kPhasePeriod;

    obs_.fill(0.0f);
    for (size_t i = 0; i < 3; ++i) {
      obs_[t170::kObsGyro + i] = static_cast<float>(state.base.angular_velocity[i]);
      obs_[t170::kObsGravity + i] = static_cast<float>(state.base.projected_gravity[i]);
      obs_[t170::kObsCmd + i] = static_cast<float>(cmd[i]);
    }
    if (cmd_norm >= 0.1) {
      obs_[t170::kObsPhase + 0] = static_cast<float>(std::sin(2.0 * M_PI * phi));
      obs_[t170::kObsPhase + 1] = static_cast<float>(std::cos(2.0 * M_PI * phi));
    }
    for (size_t i = 0; i < t170::kNumJoints; ++i) {
      obs_[t170::kObsJointPos + i] =
        static_cast<float>(state.measured_positions[index_[i]]) - t170::kQHome[i];
      obs_[t170::kObsJointVel + i] = static_cast<float>(state.measured_velocities[index_[i]]);
      obs_[t170::kObsLastAction + i] = last_action_[i];
    }

    const auto start = std::chrono::steady_clock::now();
    const int64_t in_dims[2] = {1, t170::kObsDim};
    auto input = Ort::Value::CreateTensor<float>(
      *memory_info_, obs_.data(), obs_.size(), in_dims, 2);
    const char * input_names[] = {input_name_.c_str()};
    const char * output_names[] = {output_name_.c_str()};
    auto outputs = session_->Run(
      Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
    last_inference_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();

    const float * action = outputs[0].GetTensorData<float>();
    for (size_t i = 0; i < t170::kNumJoints; ++i) {
      last_action_[i] = action[i];
      target_[i] = t170::kQHome[i] + t170::kActionScale[i] * action[i];
    }
    has_target_ = true;
    clock_ += policy_dt_;
  }

  void warn(const std::string & message) const {
    if (log_) {
      log_(LogLevel::kWarn, "T170VelocityPolicy: " + message);
    }
  }

  Logger log_;
  double policy_dt_{t170::kCtrlDt};
  double log_period_{0.0};
  std::array<size_t, t170::kNumJoints> index_{};  // policy 順序 → controller joint index

  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;
  std::string input_name_;
  std::string output_name_;

  std::array<float, t170::kObsDim> obs_{};
  std::array<float, t170::kNumJoints> last_action_{};
  std::array<double, t170::kNumJoints> target_{};
  bool has_target_{false};
  bool holding_{false};
  double clock_{0.0};
  double since_inference_{0.0};
  double since_log_{0.0};
  double last_inference_ms_{0.0};
};

}  // namespace graphen::rl_policy

PLUGINLIB_EXPORT_CLASS(
  graphen::rl_policy::T170VelocityPolicy,
  graphen::locomotion::LocomotionAlgorithm)
