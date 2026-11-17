#ifndef GRAPHEN_CONTROLLERS__POLICY_SOURCE_HPP_
#define GRAPHEN_CONTROLLERS__POLICY_SOURCE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "graphen_controllers/playback_arbiter.hpp"
#include "graphen_controllers/plugin_factory.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace graphen::control {

// Policy（mode "policy"）的來源：policy 庫中的 <library_path>/<name>.policy.yaml。
//
// 每個檔案是一個 policy 的完整設定（演算法 plugin、onnx、訓練常數），和 onnx 綁在一起、跟著程式碼版控，
// 由控制 / 訓練端管理；這裡只讀取，沒有發佈介面。檔案內容：
//   plugin: <LocomotionAlgorithm plugin 的類別名稱>      # 必填
//   max_steps: [...]                                     # （選填）每週期最大移動量，預設 planner_max_steps
//   interruptible: true                                  # （選填）執行中是否可被打斷，預設 true
//   其餘的鍵由演算法讀取（model_path、model_sha256、joints、kp、kd …）
// controller 的參數 <name>.<鍵> 會覆寫檔案中的同名鍵（情境 launch 用來調整 entry_speed 之類的值），
// 不必為了一個情境複製整份設定。
//
// 第一次使用時載入（建立演算法實例並 configure：讀 onnx、建立推論 session，可能要數百毫秒），之後快取；
// 檔案內容改變（sha256 不同）時重新載入。同一個實例會被重複進入（on_enter 會重設內部狀態）。
//
// 參數（controller 的 policy.*）：
//   policy.library_path  policy 庫目錄，支援 ~/ 與 package://；空字串表示不啟用（enabled() 為 false）
//
// 所有函式都在 non-RT thread 呼叫（內部以 mutex 保護快取）；失敗時丟出例外。
class PolicySource {
public:
  static constexpr const char * kMode = "policy";

  // 讀取 policy.* 參數；設定錯誤時丟出 std::invalid_argument。factory 必須比這個物件活得久
  PolicySource(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, PluginFactory * factory,
    const std::vector<double> & default_max_steps);

  bool enabled() const {return !library_path_.empty();}

  // name 是否已在快取中而且沒有過期（不需要載入）
  bool is_loaded(const std::string & name);

  PlaybackRequest prepare(const std::string & name);

  // 只載入到快取
  void preload(const std::string & name) {prepare(name);}

private:
  struct Loaded {
    std::string sha256;  // 設定檔的內容
    std::shared_ptr<Motion> motion;
    bool interruptible{true};
  };

  std::string path_of(const std::string & name) const;

  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  PluginFactory * const factory_;
  const std::vector<double> default_max_steps_;
  std::string library_path_;

  std::mutex mutex_;
  std::map<std::string, Loaded> cache_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__POLICY_SOURCE_HPP_
