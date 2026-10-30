#ifndef GRAPHEN_RL_POLICY__ONNX_MODEL_HPP_
#define GRAPHEN_RL_POLICY__ONNX_MODEL_HPP_

#include <memory>
#include <string>

namespace graphen::rl_policy {

// 單一輸入 [1, input_dim] → 單一輸出 [1, output_dim] 的 ONNX 模型（ONNX Runtime，CPU 單執行緒）。
//
// 建構時檢查：
//   - 檔案的 SHA-256 與 expected_sha256 相同（onnx 與 plugin 設定綁定，避免載入維度相同但
//     訓練設定不同的模型）；expected_sha256 不分大小寫
//   - 輸入輸出各只有一個，形狀為 [1, input_dim] / [1, output_dim]（第一維可為動態）
// 不符時丟出 std::invalid_argument。ORT 的型別不出現在這個 header，使用的套件不需要 ORT 的 include。
class OnnxModel {
public:
  OnnxModel(
    const std::string & path, const std::string & expected_sha256,
    size_t input_dim, size_t output_dim);
  ~OnnxModel();

  OnnxModel(const OnnxModel &) = delete;
  OnnxModel & operator=(const OnnxModel &) = delete;

  // input 長度為 input_dim，結果直接寫入 output（長度為 output_dim）。推論失敗時丟出例外
  void run(const float * input, float * output);

  // 最近一次 run() 的耗時（毫秒）
  double last_run_ms() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// package://<pkg>/<rel> → <pkg 的 share 目錄>/<rel>；其他字串原樣回傳。
// 找不到套件時丟出 std::invalid_argument
std::string resolve_package_path(const std::string & path);

}  // namespace graphen::rl_policy

#endif  // GRAPHEN_RL_POLICY__ONNX_MODEL_HPP_
