#ifndef GRAPHEN_MOTION__BVH_CLIP_HPP_
#define GRAPHEN_MOTION__BVH_CLIP_HPP_

#include <istream>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace graphen::motion {

enum class BvhChannel {
  kXposition, kYposition, kZposition,
  kXrotation, kYrotation, kZrotation,
};

struct BvhJoint {
  std::string name;
  int parent{-1};                    // 父節點在 BvhClip::joints 中的 index，root 為 -1
  Eigen::Vector3d offset{Eigen::Vector3d::Zero()};
  std::vector<BvhChannel> channels;
  size_t channel_offset{0};          // 此節點的第一個 channel 在每一幀資料中的位置
};

// 一段 BVH 動作：骨架（hierarchy）與每一幀的 channel 值。
// joints 依檔案中出現的順序排列，父節點一定在子節點之前。End Site 不列入。
struct BvhClip {
  std::vector<BvhJoint> joints;
  double frame_time{0.0};                   // 秒
  std::vector<std::vector<double>> frames;  // frames[i] 長度 = 所有 channel 數

  size_t num_frames() const {return frames.size();}

  std::optional<size_t> find_joint(const std::string & name) const;

  // 依 channel 的列出順序相乘旋轉（例如 Zrotation Xrotation Yrotation → Rz·Rx·Ry），單位為度。
  // 位移 channel 會被忽略
  Eigen::Matrix3d local_rotation(size_t frame, size_t joint) const;
};

// 解析失敗時丟出 std::runtime_error
BvhClip parse_bvh(std::istream & in);
BvhClip parse_bvh_file(const std::string & path);

}  // namespace graphen::motion

#endif  // GRAPHEN_MOTION__BVH_CLIP_HPP_
