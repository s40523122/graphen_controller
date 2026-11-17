#include "graphen_controllers/parameter_sources.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace graphen::control {

namespace {

void expect(const rclcpp::Parameter & param, rclcpp::ParameterType type) {
  if (param.get_type() != type) {
    throw std::invalid_argument(
            "parameter '" + param.get_name() + "' has type " +
            rclcpp::to_string(param.get_type()) + ", expected " + rclcpp::to_string(type));
  }
}

// 依序問每一層，回傳第一個有值的
template<typename Getter>
auto first_value(
  const std::vector<const locomotion::ParameterSource *> & layers, Getter getter,
  const std::string & name) -> decltype((layers.front()->*getter)(name))
{
  for (const auto * layer : layers) {
    if (auto value = (layer->*getter)(name)) {
      return value;
    }
  }
  return std::nullopt;
}

}  // namespace

NodeParameterSource::NodeParameterSource(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, std::string prefix)
: node_(std::move(node)), prefix_(std::move(prefix)) {}

std::optional<rclcpp::Parameter> NodeParameterSource::find(const std::string & name) const {
  const auto full_name = prefix_ + name;
  if (!node_->has_parameter(full_name)) {
    return std::nullopt;
  }
  auto param = node_->get_parameter(full_name);
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return std::nullopt;
  }
  return param;
}

std::optional<bool> NodeParameterSource::get_bool(const std::string & name) const {
  const auto param = find(name);
  if (!param) {
    return std::nullopt;
  }
  expect(*param, rclcpp::ParameterType::PARAMETER_BOOL);
  return param->as_bool();
}

std::optional<int64_t> NodeParameterSource::get_int(const std::string & name) const {
  const auto param = find(name);
  if (!param) {
    return std::nullopt;
  }
  expect(*param, rclcpp::ParameterType::PARAMETER_INTEGER);
  return param->as_int();
}

std::optional<double> NodeParameterSource::get_double(const std::string & name) const {
  const auto param = find(name);
  if (!param) {
    return std::nullopt;
  }
  // YAML 寫成 1 而不是 1.0 時會是整數
  if (param->get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(param->as_int());
  }
  expect(*param, rclcpp::ParameterType::PARAMETER_DOUBLE);
  return param->as_double();
}

std::optional<std::string> NodeParameterSource::get_string(const std::string & name) const {
  const auto param = find(name);
  if (!param) {
    return std::nullopt;
  }
  expect(*param, rclcpp::ParameterType::PARAMETER_STRING);
  return param->as_string();
}

std::optional<std::vector<double>> NodeParameterSource::get_double_array(
  const std::string & name) const
{
  const auto param = find(name);
  if (!param) {
    return std::nullopt;
  }
  if (param->get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    const auto values = param->as_integer_array();
    return std::vector<double>(values.begin(), values.end());
  }
  expect(*param, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  return param->as_double_array();
}

std::optional<std::vector<std::string>> NodeParameterSource::get_string_array(
  const std::string & name) const
{
  const auto param = find(name);
  if (!param) {
    return std::nullopt;
  }
  expect(*param, rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  return param->as_string_array();
}

YamlParameterSource::YamlParameterSource(YAML::Node map, std::string origin)
: map_(std::move(map)), origin_(std::move(origin)) {
  if (!map_.IsMap()) {
    throw std::invalid_argument(origin_ + ": expected a YAML map of parameters");
  }
}

YAML::Node YamlParameterSource::find(const std::string & name) const {
  // 對 const node 使用 operator[] 不會新增鍵
  const auto node = map_[name];
  if (!node.IsDefined() || node.IsNull()) {
    return YAML::Node(YAML::NodeType::Undefined);
  }
  return node;
}

void YamlParameterSource::fail(const std::string & name, const std::string & expected) const {
  throw std::invalid_argument("parameter '" + name + "' in " + origin_ + ": expected " + expected);
}

std::optional<bool> YamlParameterSource::get_bool(const std::string & name) const {
  const auto node = find(name);
  if (!node.IsDefined()) {
    return std::nullopt;
  }
  // 只接受 true / false，與 ROS 參數檔的寫法一致
  if (!node.IsScalar() || (node.Scalar() != "true" && node.Scalar() != "false")) {
    fail(name, "true or false");
  }
  return node.Scalar() == "true";
}

std::optional<int64_t> YamlParameterSource::get_int(const std::string & name) const {
  const auto node = find(name);
  if (!node.IsDefined()) {
    return std::nullopt;
  }
  int64_t value = 0;
  if (!node.IsScalar() || !YAML::convert<int64_t>::decode(node, value)) {
    fail(name, "an integer");
  }
  return value;
}

std::optional<double> YamlParameterSource::get_double(const std::string & name) const {
  const auto node = find(name);
  if (!node.IsDefined()) {
    return std::nullopt;
  }
  double value = 0.0;
  if (!node.IsScalar() || !YAML::convert<double>::decode(node, value)) {
    fail(name, "a number");
  }
  return value;
}

std::optional<std::string> YamlParameterSource::get_string(const std::string & name) const {
  const auto node = find(name);
  if (!node.IsDefined()) {
    return std::nullopt;
  }
  if (!node.IsScalar()) {
    fail(name, "a string");
  }
  return node.Scalar();
}

std::optional<std::vector<double>> YamlParameterSource::get_double_array(
  const std::string & name) const
{
  const auto node = find(name);
  if (!node.IsDefined()) {
    return std::nullopt;
  }
  if (!node.IsSequence()) {
    fail(name, "a list of numbers");
  }
  std::vector<double> values;
  values.reserve(node.size());
  for (const auto & entry : node) {
    double value = 0.0;
    if (!entry.IsScalar() || !YAML::convert<double>::decode(entry, value)) {
      fail(name, "a list of numbers");
    }
    values.push_back(value);
  }
  return values;
}

std::optional<std::vector<std::string>> YamlParameterSource::get_string_array(
  const std::string & name) const
{
  const auto node = find(name);
  if (!node.IsDefined()) {
    return std::nullopt;
  }
  if (!node.IsSequence()) {
    fail(name, "a list of strings");
  }
  std::vector<std::string> values;
  values.reserve(node.size());
  for (const auto & entry : node) {
    if (!entry.IsScalar()) {
      fail(name, "a list of strings");
    }
    values.push_back(entry.Scalar());
  }
  return values;
}

OverlayParameterSource::OverlayParameterSource(
  std::vector<const locomotion::ParameterSource *> layers)
: layers_(std::move(layers)) {}

std::optional<bool> OverlayParameterSource::get_bool(const std::string & name) const {
  return first_value(layers_, &locomotion::ParameterSource::get_bool, name);
}

std::optional<int64_t> OverlayParameterSource::get_int(const std::string & name) const {
  return first_value(layers_, &locomotion::ParameterSource::get_int, name);
}

std::optional<double> OverlayParameterSource::get_double(const std::string & name) const {
  return first_value(layers_, &locomotion::ParameterSource::get_double, name);
}

std::optional<std::string> OverlayParameterSource::get_string(const std::string & name) const {
  return first_value(layers_, &locomotion::ParameterSource::get_string, name);
}

std::optional<std::vector<double>> OverlayParameterSource::get_double_array(
  const std::string & name) const
{
  return first_value(layers_, &locomotion::ParameterSource::get_double_array, name);
}

std::optional<std::vector<std::string>> OverlayParameterSource::get_string_array(
  const std::string & name) const
{
  return first_value(layers_, &locomotion::ParameterSource::get_string_array, name);
}

}  // namespace graphen::control
