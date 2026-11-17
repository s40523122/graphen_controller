#ifndef GRAPHEN_CONTROLLERS__LIBRARY_UTILS_HPP_
#define GRAPHEN_CONTROLLERS__LIBRARY_UTILS_HPP_

#include <string>

namespace graphen::control {

// 內容庫（舞曲庫、BVH 目錄、policy 庫）共用的小工具。

// 把設定中的路徑轉成實際路徑：
//   package://<pkg>/<path>  → 該套件安裝後的 share 目錄下的路徑
//   ~/<path>                → $HOME/<path>
//   其他                    → 原樣回傳
// 格式錯誤、找不到套件或沒有 HOME 時丟出 std::invalid_argument
std::string resolve_path(const std::string & path);

// 項目名稱會直接組成檔名（<目錄>/<name>.<副檔名>），只允許 [a-z0-9_-]、1 ~ 64 個字元：
// 擋掉路徑穿越（../）與絕對路徑
bool is_valid_item_name(const std::string & name);

// is_valid_item_name() 不通過時丟出 std::invalid_argument（kind 用在訊息中，例如 "choreography"）
void require_valid_item_name(const std::string & kind, const std::string & name);

// 讀取整個檔案；無法開啟時丟出 std::runtime_error
std::string read_text_file(const std::string & path);

// sha256 的前 12 個字元，用在 log 中辨識內容的版本
std::string short_hash(const std::string & sha256);

}  // namespace graphen::control

#endif  // GRAPHEN_CONTROLLERS__LIBRARY_UTILS_HPP_
