#include "graphen_rl_policy/onnx_model.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "graphen_locomotion/sha256.hpp"
#include "onnxruntime_cxx_api.h"

namespace graphen::rl_policy {

namespace {

std::string to_lower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return text;
}

// [1, dim]，第一維允許動態（-1）
bool is_row_vector(const std::vector<int64_t> & shape, size_t dim) {
  return shape.size() == 2 && (shape[0] == 1 || shape[0] == -1) &&
         shape[1] == static_cast<int64_t>(dim);
}

std::string shape_to_string(const std::vector<int64_t> & shape) {
  std::string text = "[";
  for (size_t i = 0; i < shape.size(); ++i) {
    text += (i > 0 ? ", " : "") + std::to_string(shape[i]);
  }
  return text + "]";
}

}  // namespace

struct OnnxModel::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "graphen_rl_policy"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo memory_info{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  std::string input_name;
  std::string output_name;
  std::array<int64_t, 2> input_shape{1, 0};
  std::array<int64_t, 2> output_shape{1, 0};
  double last_run_ms{0.0};
};

OnnxModel::OnnxModel(
  const std::string & path, const std::string & expected_sha256,
  size_t input_dim, size_t output_dim)
: impl_(std::make_unique<Impl>())
{
  if (expected_sha256.empty()) {
    throw std::invalid_argument("'model_sha256' is required (sha256sum of the onnx file)");
  }
  std::string actual_sha256;
  try {
    actual_sha256 = locomotion::Sha256::file_hex_digest(path);
  } catch (const std::exception & e) {
    throw std::invalid_argument(std::string("cannot read model: ") + e.what());
  }
  if (actual_sha256 != to_lower(expected_sha256)) {
    throw std::invalid_argument(
      "model '" + path + "' has sha256 " + actual_sha256 + ", expected " + expected_sha256 +
      " (the onnx does not belong to this configuration)");
  }

  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(1);  // 小型 MLP：單執行緒最快且結果固定
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  try {
    impl_->session = std::make_unique<Ort::Session>(impl_->env, path.c_str(), options);
  } catch (const Ort::Exception & e) {
    throw std::invalid_argument("cannot load '" + path + "': " + e.what());
  }

  auto & session = *impl_->session;
  if (session.GetInputCount() != 1 || session.GetOutputCount() != 1) {
    throw std::invalid_argument(
      "model '" + path + "' must have exactly one input and one output");
  }
  Ort::AllocatorWithDefaultOptions allocator;
  impl_->input_name = session.GetInputNameAllocated(0, allocator).get();
  impl_->output_name = session.GetOutputNameAllocated(0, allocator).get();
  const auto in_shape = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  const auto out_shape = session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  if (!is_row_vector(in_shape, input_dim) || !is_row_vector(out_shape, output_dim)) {
    throw std::invalid_argument(
      "model '" + path + "' maps " + shape_to_string(in_shape) + " -> " +
      shape_to_string(out_shape) + ", the plugin expects [1, " + std::to_string(input_dim) +
      "] -> [1, " + std::to_string(output_dim) + "]");
  }
  impl_->input_shape[1] = static_cast<int64_t>(input_dim);
  impl_->output_shape[1] = static_cast<int64_t>(output_dim);
}

OnnxModel::~OnnxModel() = default;

void OnnxModel::run(const float * input, float * output) {
  const auto start = std::chrono::steady_clock::now();
  // ORT 的 CreateTensor 需要非 const 指標，但只會讀取輸入
  auto input_tensor = Ort::Value::CreateTensor<float>(
    impl_->memory_info, const_cast<float *>(input),
    static_cast<size_t>(impl_->input_shape[1]), impl_->input_shape.data(),
    impl_->input_shape.size());
  // 輸出直接寫到呼叫端的緩衝，不另外配置
  auto output_tensor = Ort::Value::CreateTensor<float>(
    impl_->memory_info, output, static_cast<size_t>(impl_->output_shape[1]),
    impl_->output_shape.data(), impl_->output_shape.size());
  const char * input_names[] = {impl_->input_name.c_str()};
  const char * output_names[] = {impl_->output_name.c_str()};
  impl_->session->Run(
    Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, &output_tensor, 1);
  impl_->last_run_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

double OnnxModel::last_run_ms() const {
  return impl_->last_run_ms;
}

std::string resolve_package_path(const std::string & path) {
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

}  // namespace graphen::rl_policy
