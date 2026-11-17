#ifndef GRAPHEN_CONTROLLERS__CHOREO_SOURCE_HPP_
#define GRAPHEN_CONTROLLERS__CHOREO_SOURCE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "graphen_controllers/playback_arbiter.hpp"
#include "graphen_controllers/plugin_factory.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace graphen::control {

// 編舞（mode "choreo"）的來源：機器人端的舞曲庫，以及把一份腳本變成可以播放的 Motion。
//
// 舞曲庫是一個目錄，每段已發佈的編舞是 <library_path>/<name>.choreo.yaml。編輯與保存在外部工具
// （UI）進行；這裡只保存「發佈過的版本」，同名覆寫、不保留歷史，也不提供刪除。
// 腳本的解析與檢查由演算法 plugin（choreo.plugin，通常是 graphen_choreographer/ChoreographerAlgorithm）
// 負責：每次播放、發佈都重新建立一個實例並 configure，所以
//   - 發佈與播放用的是同一套檢查（格式、限位、速度、max_steps、姿態的 joint 名稱）
//   - 播放中的實例持有已解析的內容，重新發佈不影響正在播的那一次
// 機器人端的上限（choreo.max_steps、choreo.velocity_limit_scale 等）只來自 controller 的參數，
// 腳本無法放寬。
//
// 參數（controller 的 choreo.*）：
//   choreo.plugin        演算法 plugin 的類別名稱；空字串表示不啟用編舞（enabled() 為 false）
//   choreo.library_path  舞曲庫目錄，支援 ~/ 與 package://
//   choreo.max_steps     （選填）每週期最大移動量，所有編舞共用；預設為 planner_max_steps
//   其餘 choreo.*        由 plugin 讀取（velocity_limit_scale、start_tolerance、entry_velocity）
//
// 所有函式都在 non-RT thread 呼叫；內容有誤、檔案不存在等一律丟出例外（what() 可以直接回給使用者）。
class ChoreoSource {
public:
  static constexpr const char * kMode = "choreo";

  struct Entry {
    std::string name;
    std::string sha256;
  };

  // 讀取 choreo.* 參數；設定錯誤時丟出 std::invalid_argument。factory 必須比這個物件活得久
  ChoreoSource(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, PluginFactory * factory,
    const std::vector<double> & default_max_steps);

  bool enabled() const {return !plugin_.empty();}

  // 舞曲庫中已發佈的編舞
  PlaybackRequest prepare(const std::string & name) const;
  // 直接以腳本內容試播（不存檔）
  PlaybackRequest prepare_preview(const std::string & content) const;

  // 完整檢查後寫入舞曲庫（同名覆寫），回傳內容的 sha256
  std::string publish(const std::string & name, const std::string & content) const;

  // 依名稱排序
  std::vector<Entry> list() const;

  // 讀回已發佈的內容；sha256 為輸出
  std::string get(const std::string & name, std::string & sha256) const;

private:
  std::string path_of(const std::string & name) const;
  PlaybackRequest build(
    const std::string & item, const std::string & content, const std::string & source) const;

  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  PluginFactory * const factory_;
  std::string plugin_;
  std::string library_path_;
  std::vector<double> max_steps_;
};

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__CHOREO_SOURCE_HPP_
