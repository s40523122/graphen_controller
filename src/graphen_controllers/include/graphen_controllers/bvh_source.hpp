#ifndef GRAPHEN_CONTROLLERS__BVH_SOURCE_HPP_
#define GRAPHEN_CONTROLLERS__BVH_SOURCE_HPP_

#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "graphen_controllers/playback_arbiter.hpp"
#include "graphen_motion/bvh_retargeter.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace graphen::control {

// BVH 動作（mode "bvh"）的來源：一個放 BVH 檔的目錄，播放 <library_path>/<name>.bvh。
//
// 檔案由人手動放進目錄（沒有上傳介面）。解析後的 BvhClip 與 BvhRetargeter 會快取（以檔案的修改時間
// 判斷是否過期），重複播放、或先用 preload() 載入之後，播放不必再讀檔。每次播放都建立新的 BvhMotion
// （播放選項 loop / time_scale 來自指令），快取的 clip / retargeter 是唯讀的，可以共用。
// BVH 沒有腳本層的設定：每週期上限與是否可中斷都由 controller 的參數決定，對所有 BVH 一致。
//
// 參數（controller 的 bvh.*）：
//   bvh.library_path        BVH 目錄，支援 ~/ 與 package://；空字串表示不啟用 BVH（enabled() 為 false）
//   bvh.max_steps           （選填）播放時每週期的最大移動量；預設為 planner_max_steps。
//                           一幀會一次跳到位，要讓一個週期就能走完一幀（見 bvh_retarget_cli）
//   bvh.approach_max_steps  （選填）走向第 0 幀時的每週期最大移動量；預設為 planner_max_steps
//   bvh.interruptible       （選填，預設 true）false 時 BVH 播放中不可被打斷（循環播放就只有停止類能切入）
//   bvh.base_link / axis_tolerance / max_iterations / damping / residual_tolerance
//                           BvhRetargeter 的分組與求解設定（base_link 必填）
// 另外需要 robot_description。
//
// 所有函式都在 non-RT thread 呼叫（內部以 mutex 保護快取）；失敗時丟出例外。
class BvhSource {
public:
  static constexpr const char * kMode = "bvh";

  // 讀取 bvh.* 參數；設定錯誤時丟出 std::invalid_argument
  BvhSource(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    std::vector<std::string> joint_names, const std::vector<double> & default_max_steps);

  bool enabled() const {return !library_path_.empty();}

  // name 是否已在快取中而且沒有過期（不需要載入）
  bool is_loaded(const std::string & name);

  // time_scale 必須 > 0
  PlaybackRequest prepare(const std::string & name, bool loop, double time_scale);

  // 只載入到快取
  void preload(const std::string & name);

private:
  struct Loaded {
    std::filesystem::file_time_type modified;
    std::shared_ptr<const motion::BvhRetargeter> retargeter;
  };

  std::string path_of(const std::string & name) const;
  // 從快取取得，沒有或過期時載入。呼叫端需持有 mutex_
  std::shared_ptr<const motion::BvhRetargeter> load(const std::string & name);

  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  const std::vector<std::string> joint_names_;
  std::string library_path_;
  std::vector<double> max_steps_;
  std::vector<double> approach_max_steps_;
  bool interruptible_{true};
  motion::BvhRetargeter::Options retarget_options_;
  std::string robot_description_;

  std::mutex mutex_;
  std::map<std::string, Loaded> cache_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__BVH_SOURCE_HPP_
