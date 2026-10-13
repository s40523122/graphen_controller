#ifndef GRAPHEN_LOCOMOTION__PARAMETER_SOURCE_HPP_
#define GRAPHEN_LOCOMOTION__PARAMETER_SOURCE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace graphen::locomotion {

// 演算法讀取參數的介面，讓演算法不必依賴 rclcpp。
//
// 由 controller 實作：名稱會自動加上 "<mode>." 前綴，從 controller 的參數（YAML）中讀取。
// 參數不存在時回傳 std::nullopt；存在但型別不符時丟出 std::invalid_argument。
// get_double / get_double_array 也接受整數值（YAML 寫成 1 而不是 1.0 時）。
class ParameterSource {
public:
  virtual ~ParameterSource() = default;

  virtual std::optional<bool> get_bool(const std::string & name) const = 0;
  virtual std::optional<int64_t> get_int(const std::string & name) const = 0;
  virtual std::optional<double> get_double(const std::string & name) const = 0;
  virtual std::optional<std::string> get_string(const std::string & name) const = 0;
  virtual std::optional<std::vector<double>> get_double_array(const std::string & name) const = 0;
  virtual std::optional<std::vector<std::string>> get_string_array(
    const std::string & name) const = 0;
};

}  // namespace graphen::locomotion

#endif  // GRAPHEN_LOCOMOTION__PARAMETER_SOURCE_HPP_
