#ifndef GRAPHEN_CONTROLLERS__PARAMETER_SOURCES_HPP_
#define GRAPHEN_CONTROLLERS__PARAMETER_SOURCES_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "graphen_locomotion/parameter_source.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "yaml-cpp/yaml.h"

namespace graphen::control {

// locomotion::ParameterSource 的幾種實作：演算法 plugin 不依賴 rclcpp，參數一律透過這個介面取得。
// 共同的規則：參數不存在時回傳 nullopt；存在但型別不符時丟出 std::invalid_argument。
// get_double / get_double_array 也接受整數值（YAML 寫成 1 而不是 1.0 時）。

// controller node 的 <prefix><name> 參數。
// 只讀取已宣告的參數（YAML 有寫的會被 controller node 自動宣告），沒有的回傳 nullopt
class NodeParameterSource : public locomotion::ParameterSource {
public:
  NodeParameterSource(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, std::string prefix);

  std::optional<bool> get_bool(const std::string & name) const override;
  std::optional<int64_t> get_int(const std::string & name) const override;
  std::optional<double> get_double(const std::string & name) const override;
  std::optional<std::string> get_string(const std::string & name) const override;
  std::optional<std::vector<double>> get_double_array(const std::string & name) const override;
  std::optional<std::vector<std::string>> get_string_array(
    const std::string & name) const override;

private:
  std::optional<rclcpp::Parameter> find(const std::string & name) const;

  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  const std::string prefix_;
};

// 一個 YAML map 的頂層鍵（例如 policy 設定檔）。origin 只用在錯誤訊息中（例如檔案路徑）
class YamlParameterSource : public locomotion::ParameterSource {
public:
  // map 必須是 YAML map，否則丟出 std::invalid_argument
  YamlParameterSource(YAML::Node map, std::string origin);

  std::optional<bool> get_bool(const std::string & name) const override;
  std::optional<int64_t> get_int(const std::string & name) const override;
  std::optional<double> get_double(const std::string & name) const override;
  std::optional<std::string> get_string(const std::string & name) const override;
  std::optional<std::vector<double>> get_double_array(const std::string & name) const override;
  std::optional<std::vector<std::string>> get_string_array(
    const std::string & name) const override;

private:
  // 鍵不存在或值為 null 時回傳未定義的 node
  YAML::Node find(const std::string & name) const;
  [[noreturn]] void fail(const std::string & name, const std::string & expected) const;

  const YAML::Node map_;
  const std::string origin_;
};

// 依序查詢多個來源，回傳第一個有值的（前面的覆寫後面的）。來源必須比這個物件活得久
class OverlayParameterSource : public locomotion::ParameterSource {
public:
  explicit OverlayParameterSource(std::vector<const locomotion::ParameterSource *> layers);

  std::optional<bool> get_bool(const std::string & name) const override;
  std::optional<int64_t> get_int(const std::string & name) const override;
  std::optional<double> get_double(const std::string & name) const override;
  std::optional<std::string> get_string(const std::string & name) const override;
  std::optional<std::vector<double>> get_double_array(const std::string & name) const override;
  std::optional<std::vector<std::string>> get_string_array(
    const std::string & name) const override;

private:
  const std::vector<const locomotion::ParameterSource *> layers_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__PARAMETER_SOURCES_HPP_
